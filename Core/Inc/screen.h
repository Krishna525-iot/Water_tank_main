#ifndef SCREEN_H
#define SCREEN_H

#include "lcd_i2c.h" // Include your LCD driver
#include "adc.h"     // Include ADC data structure
#include "model_handle.h" // Include model handle for mode status

// Declare external variables
extern ADC_Data adcData; // Assuming this is defined in adc.h

// Define LCD screen states
typedef enum {
    LCD_STATE_WELCOME,
    LCD_STATE_HOME_WATER_LEVEL,
    LCD_STATE_TIMER_MODE,
    LCD_STATE_SEARCH_MODE,
    LCD_STATE_COUNTDOWN_MODE,
    LCD_STATE_TWIST_MODE,
    LCD_STATE_MODE_STATUS // For Manual, Semi-Auto, Search, Countdown ON/OFF
} LcdState;

// Function prototypes
void Screen_Init(void);
void Screen_Update(void);
void Screen_SetState(LcdState state);
// Countdown (repeatable) mode API
void ModelHandle_StartCountdown(uint32_t seconds_per_run);
void ModelHandle_StopCountdown(void);
void Screen_HandleSwitches(void);
// Exposed so the LCD can show remaining runs
extern volatile uint16_t countdownRemainingRuns;

#endif // SCREEN_H
