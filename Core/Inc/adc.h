#ifndef __ADC_H
#define __ADC_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

#define ADC_CHANNEL_COUNT 6

/* Struct to hold ADC readings */
typedef struct {
    uint16_t rawValues[ADC_CHANNEL_COUNT];
    float    voltages[ADC_CHANNEL_COUNT];
    uint8_t  maxReached[ADC_CHANNEL_COUNT];
} ADC_Data;

/* Public functions */
void ADC_Init(ADC_HandleTypeDef* hadc);
void ADC_ReadAllChannels(ADC_HandleTypeDef* hadc, ADC_Data* data);
uint8_t ADC_CheckMaxVoltage(ADC_Data* data, float threshold);

#endif
