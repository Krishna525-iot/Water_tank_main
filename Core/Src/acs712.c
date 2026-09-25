#include "acs712.h"
#include "eeprom_i2c.h"
#include "math.h"
#include <string.h>

float g_currentA = 0.0f;
float g_voltageV = 0.0f;
float g_powerW;
static ADC_HandleTypeDef *hAdc;
float adc_rms;
static float acs_zero_offset = 0.0f;
static float zmpt_offset = 1.65f;
static float last_voltage = 0.0f;
static float last_current = 0.0f;
static float last_raw_current = 0.0f;
#define ACS712_GAIN_CORR  1.25f
#define ACS712_NOISE_DEADBAND_A  0.10f   /* readings below this (while motor is ON) are treated as noise */

/* -------------------------------------------------------
   CALIBRATION (kept in EEPROM, set from the app)
     CAL:V:<volts>  - scale voltage to a meter reading
     CAL:I:<amps>   - scale current to a clamp-meter reading
     CAL:I:0        - motor on with no load: learn the zero
-------------------------------------------------------- */
#define EE_ADDR_CAL_BLOCK  0x0900
#define CAL_SIG            0xCA1B

typedef struct __attribute__((packed)) {
    uint16_t sig;
    float    vFactor;
    float    iGain;
    uint16_t crc;
} CalBlock;

static float zmpt_factor  = ZMPT_CALIBRATION;
static float acs_gain     = ACS712_GAIN_CORR;
static float acs_noise_a  = 0.0f;   /* RMS noise floor, subtracted in quadrature */

extern volatile uint8_t motorStatus;   /* live relay state - 1=on, 0=off */

static uint16_t cal_crc(const uint8_t *d, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    while (len--)
    {
        crc ^= *d++;
        for (uint8_t i = 0; i < 8; i++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
    return crc;
}

static void cal_save(void)
{
    CalBlock b;
    memset(&b, 0, sizeof(b));
    b.sig     = CAL_SIG;
    b.vFactor = zmpt_factor;
    b.iGain   = acs_gain;
    b.crc     = cal_crc((uint8_t*)&b, sizeof(b) - 2);
    EEPROM_WriteBuffer(EE_ADDR_CAL_BLOCK, (uint8_t*)&b, sizeof(b));
}

static void cal_load(void)
{
    CalBlock b;
    EEPROM_ReadBuffer(EE_ADDR_CAL_BLOCK, (uint8_t*)&b, sizeof(b));
    if (b.sig != CAL_SIG || b.crc != cal_crc((uint8_t*)&b, sizeof(b) - 2)) return;
    if (b.vFactor > 50.0f && b.vFactor < 1000.0f) zmpt_factor = b.vFactor;
    if (b.iGain   > 0.1f  && b.iGain   < 10.0f)   acs_gain    = b.iGain;
}

/* -------------------------------------------------------
   ADC READER
-------------------------------------------------------- */
static float adc_read(uint32_t channel)
{
    ADC_ChannelConfTypeDef cfg = {0};
    cfg.Channel = channel;
    cfg.Rank = ADC_REGULAR_RANK_1;
    cfg.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;

    HAL_ADC_ConfigChannel(hAdc, &cfg);

    HAL_ADC_Start(hAdc);
    HAL_ADC_PollForConversion(hAdc, HAL_MAX_DELAY);

    uint16_t raw = HAL_ADC_GetValue(hAdc);
    HAL_ADC_Stop(hAdc);

    return (raw * ADC_VREF) / ADC_RES;
}

/* -------------------------------------------------------
   OFFSET CALIBRATION (Voltage)
-------------------------------------------------------- */
static void zmpt_calibrate_offset(void)
{
    float sum = 0;
    for (int i = 0; i < ZMPT_OFFSET_SAMPLES; i++)
        sum += adc_read(ZMPT_ADC_CHANNEL);

    zmpt_offset = sum / ZMPT_OFFSET_SAMPLES;
}

/* -------------------------------------------------------
   OFFSET CALIBRATION (Current)
-------------------------------------------------------- */
static void acs_calibrate_offset(void)
{
    float sum = 0;
    for (int i = 0; i < ACS712_ZERO_SAMPLES; i++)
        sum += adc_read(ACS712_ADC_CHANNEL);

    acs_zero_offset = sum / ACS712_ZERO_SAMPLES;
}

/* True-RMS current in amps before noise removal */
static float acs_read_raw_rms_a(void)
{
    const uint16_t SAMPLES = 2000;

    float sum_dc = 0.0f;
    float sum_sq = 0.0f;

    for (uint16_t i = 0; i < SAMPLES; i++)
    {
        float v = adc_read(ACS712_ADC_CHANNEL);
        sum_dc += v;

        float ac = v - acs_zero_offset;
        sum_sq += ac * ac;
    }

    /* Adaptive offset correction: tracks slow temperature / supply
       drift, does NOT follow the AC waveform */
    float new_offset = sum_dc / SAMPLES;
    acs_zero_offset = (acs_zero_offset * 0.995f) + (new_offset * 0.005f);

    float rms = sqrtf(sum_sq / SAMPLES);
    last_raw_current = (rms / ACS712_SENS_30A) * acs_gain;
    return last_raw_current;
}

/* The zero-load reading is RMS noise, not an offset: it must be removed
 * in quadrature. The old code subtracted a correction measured through
 * ACS712_ReadCurrent() while the motor was off - which returns 0 - so
 * the correction was always 0 and ~0.4 A showed with no load. */
void ACS712_ZeroCurrentCalibrate(void)
{
    float sum = 0.0f;
    for (int i = 0; i < 10; i++)
        sum += acs_read_raw_rms_a();
    acs_noise_a = sum / 10.0f;
}

/* -------------------------------------------------------
   INITIALIZATION
-------------------------------------------------------- */
void ACS712_Init(ADC_HandleTypeDef *hadc)
{
    hAdc = hadc;

    HAL_Delay(500);
    cal_load();

    /* VERY IMPORTANT:
       NO LOAD must be connected here
    */
    acs_calibrate_offset();
    zmpt_calibrate_offset();
    ACS712_ZeroCurrentCalibrate();
}


/* -------------------------------------------------------
   TRUE RMS VOLTAGE READ (ZMPT101B)
-------------------------------------------------------- */
float ZMPT_ReadVoltageRMS(void)
{
    float sum_dc = 0.0f;
    float sum_sq = 0.0f;

    for (int i = 0; i < ZMPT_RMS_SAMPLES; i++)
    {
        float v = adc_read(ZMPT_ADC_CHANNEL);

        sum_dc += v;

        float ac = v - zmpt_offset;
        sum_sq += ac * ac;
    }

    float new_offset = sum_dc / ZMPT_RMS_SAMPLES;

    zmpt_offset = (zmpt_offset * 0.90f) + (new_offset * 0.10f);

    adc_rms = sqrtf(sum_sq / ZMPT_RMS_SAMPLES);

    float Vrms = adc_rms * zmpt_factor;

    /* Seed the filter on the first reading so the voltage protection
       does not see a false under-voltage while it ramps up from 0 */
    if (last_voltage <= 0.0f)
        last_voltage = Vrms;
    else
        last_voltage =
            last_voltage * (1.0f - ZMPT_FILTER_ALPHA) +
            (Vrms * ZMPT_FILTER_ALPHA);

    g_voltageV = last_voltage;
    return g_voltageV;
}

/* -------------------------------------------------------
   CURRENT READING (ACS712)
-------------------------------------------------------- */
float ACS712_ReadCurrent(void)
{
    float raw = acs_read_raw_rms_a();

    /* -------- Motor off = no load = 0A, no exceptions --------
       If the relay isn't energized there is no current path, so the
       reading is pure noise: keep learning the noise floor from it. */
    if (!motorStatus)
    {
        acs_noise_a  = (acs_noise_a * 0.9f) + (raw * 0.1f);
        last_current = 0.0f;
        g_currentA   = 0.0f;
        return 0.0f;
    }

    float sq      = raw * raw - acs_noise_a * acs_noise_a;
    float current = (sq > 0.0f) ? sqrtf(sq) : 0.0f;

    /* -------- Deadband (noise kill) -------- */
    if (current < ACS712_NOISE_DEADBAND_A)
        current = 0.0f;

    /* -------- Output smoothing -------- */
    last_current = (last_current * (1.0f - ACS712_FILTER_ALPHA)) + (current * ACS712_FILTER_ALPHA);

    g_currentA = last_current;
    return g_currentA;
}

/* -------------------------------------------------------
   UPDATE BOTH SENSOR VALUES
-------------------------------------------------------- */
void ACS712_Update(void)
{
    ACS712_ReadCurrent();
    ZMPT_ReadVoltageRMS();
    g_powerW = g_currentA * g_voltageV;
}

/* -------------------------------------------------------
   USER CALIBRATION
-------------------------------------------------------- */
bool ACS712_CalibrateVoltage(float actualVolts)
{
    if (actualVolts < 50.0f || actualVolts > 400.0f) return false;
    if (g_voltageV < 20.0f) return false;          /* no mains reading to scale */

    float f = zmpt_factor * actualVolts / g_voltageV;
    if (f <= 50.0f || f >= 1000.0f) return false;

    zmpt_factor  = f;
    last_voltage = actualVolts;
    g_voltageV   = actualVolts;
    cal_save();
    return true;
}

bool ACS712_CalibrateCurrent(float actualAmps)
{
    if (!motorStatus) return false;                 /* needs the motor running */

    if (actualAmps <= 0.0f)
    {
        /* Running with no load: whatever is read now is the zero */
        acs_noise_a  = last_raw_current;
        last_current = 0.0f;
        g_currentA   = 0.0f;
        return true;
    }
    if (actualAmps > 40.0f || g_currentA < 0.2f) return false;

    float ratio = actualAmps / g_currentA;
    float g     = acs_gain * ratio;
    if (g <= 0.1f || g >= 10.0f) return false;

    acs_gain     = g;
    acs_noise_a *= ratio;
    last_current = actualAmps;
    g_currentA   = actualAmps;
    cal_save();
    return true;
}

float ACS712_GetVoltageFactor(void) { return zmpt_factor; }
float ACS712_GetCurrentGain(void)   { return acs_gain; }
