#ifndef __ACS712_H__
#define __ACS712_H__

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* -------------------------------
 *  Global values (for display)
 * ------------------------------- */
extern float g_currentA;   // Amperes
extern float g_voltageV;   // Volts

/* -------------------- ADC CONFIG -------------------- */
#define ADC_VREF   3.3f
#define ADC_RES    4095.0f

/* ------------------ ACS712 CURRENT ------------------ */
#define ACS712_ADC_CHANNEL     ADC_CHANNEL_7
#define ACS712_ZERO_SAMPLES 500
#define ACS712_FILTER_ALPHA    0.5f   /* final smoothing; was 0.1 (several s to
                                         follow the load), now ~1 s like the
                                         voltage (24-10). Trips need 3 s. */
/* Current sensor (B-02, 24-10): ACS712ELCTR-30A, 66 mV/A at its output.
 * ACS712_DIVIDER = what reaches PA7: 1.0 with no divider (good to ~8 A
 * RMS - the output tops 3.3 V above that); 0.6667f with the 10k/20k
 * divider (to 15 A+). Change only this line when the divider is fitted. */
#define ACS712_SENS_V_PER_A    0.066f
#define ACS712_DIVIDER         1.0f
#define ACS712_SENS            (ACS712_SENS_V_PER_A * ACS712_DIVIDER)

/* ---------------- ZMPT101B VOLTAGE ------------------ */
#define ZMPT_ADC_CHANNEL       ADC_CHANNEL_6
#define ZMPT_OFFSET_SAMPLES    300
#define ZMPT_RMS_SAMPLES       2000    /* >2 mains cycles: RMS over whole cycles */
#define ZMPT_FILTER_ALPHA      0.5f    /* was 0.15: took 5-7 s to follow the mains;
                                          now ~1 s, like a meter (client 16-10).
                                          Trips still need 3 s confirm. */

/* -----------------------------------------------------
   Default voltage factor (Vrms = ADC_RMS * factor). Used until the
   device is calibrated with CAL:V:<volts>, which stores its own
   factor in EEPROM.
------------------------------------------------------ */
#define ZMPT_CALIBRATION       239.5f

/* -------------------------------
 *  Function Prototypes
 * ------------------------------- */
#include <stdbool.h>

void ACS712_Init(ADC_HandleTypeDef *hadc);
void ACS712_Update(void);

float ACS712_ReadCurrent(void);
float ZMPT_ReadVoltageRMS(void);

bool  ACS712_CalibrateVoltage(float actualVolts);
bool  ACS712_CalibrateCurrent(float actualAmps);   /* 0 = zero with no load */
float ACS712_GetVoltageFactor(void);
float ACS712_GetCurrentGain(void);

#endif /* __ACS712_H__ */
