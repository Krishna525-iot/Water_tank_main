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
#define ACS712_FILTER_ALPHA    0.1f   // used only for final smoothing
#define ACS712_SENS_30A  0.066f   // 66mV per Ampere for ACS712-30A

/* ---------------- ZMPT101B VOLTAGE ------------------ */
#define ZMPT_ADC_CHANNEL       ADC_CHANNEL_6
#define ZMPT_OFFSET_SAMPLES    300
#define ZMPT_RMS_SAMPLES       800
#define ZMPT_FILTER_ALPHA      0.15f

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
