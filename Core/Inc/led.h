#ifndef LED_H
#define LED_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Colors available on your board (LED1..LED4) */
typedef enum {
    LED_COLOR_GREEN = 0,  // map to LED1
    LED_COLOR_RED,        // map to LED2
    LED_COLOR_BLUE,       // map to LED3
    LED_COLOR_PURPLE,     // map to LED4  (logical name)
    LED_COLOR_COUNT
} LedColor;

typedef enum {
    LED_MODE_OFF = 0,
    LED_MODE_STEADY,
    LED_MODE_BLINK
} LedMode;

/* Runtime */
void LED_Init(void);
void LED_Task(void);                 // call often (e.g. every loop)
void LED_All_Off(void);

/* Intent-style control (non-blocking patterns) */
void LED_ClearAllIntents(void);
void LED_SetIntent(LedColor color, LedMode mode, uint16_t period_ms);
void LED_ApplyIntents(void);

#ifdef __cplusplus
}
#endif

#endif /* LED_H */
