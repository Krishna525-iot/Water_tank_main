#include "acs712.h"
#include "math.h"

float g_currentA = 0.0f;
float g_voltageV = 0.0f;
float g_powerW;
static ADC_HandleTypeDef *hAdc;
float adc_rms;
static float acs_zero_offset = 0.0f;
static float zmpt_offset = 1.65f;
static float last_voltage = 0.0f;
static float last_current = 0.0f;
static float current_zero_correction = 0.0f;
#define ACS712_GAIN_CORR  1.25f

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
void ACS712_ZeroCurrentCalibrate(void)
{
    float sum = 0.0f;

    for (int i = 0; i < 200; i++)
        sum += ACS712_ReadCurrent();

    current_zero_correction = sum / 200.0f;
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

/* -------------------------------------------------------
   INITIALIZATION
-------------------------------------------------------- */
void ACS712_Init(ADC_HandleTypeDef *hadc)
{
    hAdc = hadc;

    HAL_Delay(500);

    /* VERY IMPORTANT:
       NO LOAD must be connected here
    */
    acs_calibrate_offset();
    zmpt_calibrate_offset();
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

    /* --------------------------
       DEBUG FOR PERFECT CALIB
       (DO NOT REMOVE NOW)
    --------------------------- */

    /* New calculation using multimeter voltage */
    #define ZMPT_CALIBRATION 239.5f  // Updated calibration factor (Multimeter RMS = 5.0 V)

    float Vrms = adc_rms * ZMPT_CALIBRATION;

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
    const uint16_t SAMPLES = 2000;

    float sum_dc = 0.0f;
    float sum_sq = 0.0f;

    /* -------- ADC sampling -------- */
    for (uint16_t i = 0; i < SAMPLES; i++)
    {
        float v = adc_read(ACS712_ADC_CHANNEL);
        sum_dc += v;

        float ac = v - acs_zero_offset;
        sum_sq += ac * ac;
    }

    /* -------- Adaptive offset correction --------
       Tracks slow temperature / supply drift
       Does NOT follow AC waveform
    */
    float new_offset = sum_dc / SAMPLES;
    acs_zero_offset = (acs_zero_offset * 0.995f) + (new_offset * 0.005f);

    /* -------- True RMS calculation -------- */
    float adc_rms = sqrtf(sum_sq / SAMPLES);

    /* -------- Convert to Amperes -------- */
    float current = (adc_rms / ACS712_SENS_30A) * ACS712_GAIN_CORR;


    /* -------- Remove zero-load bias --------
       Your system shows ~0.7 A offset at no load
       This removes it permanently
    */
    static float zero_current_corr = 0.0f;
    static uint8_t zero_cal_done = 0;

    if (!zero_cal_done)
    {
        /* Auto-zero once at startup (NO LOAD required) */
        zero_current_corr = current;
        zero_cal_done = 1;
    }

    current -= zero_current_corr;

    /* -------- Deadband (noise kill) -------- */
    if (current < 0.05f)
        current = 0.0f;

    /* -------- Output smoothing -------- */
    last_current = (last_current * 0.878f) + (current * 0.054f);



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
