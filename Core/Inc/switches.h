#pragma once
#include <stdbool.h>
#include <stdint.h>

/* Events reported per switch */
typedef enum {
    SWITCH_EVT_NONE = 0,
    SWITCH_EVT_SHORT,   // released before long threshold
    SWITCH_EVT_LONG     // long threshold crossed (fires once while held)
} SwitchEvent;

/* Change long press threshold (default 2000 ms) */
void     Switches_SetLongPressMs(uint16_t ms);

void Switches_Init(void);

/* Called every 1 ms from SysTick_Handler: samples and debounces all
 * switches and queues their events, so a quick tap is never lost while
 * the main loop is busy (ADC sampling, LCD, EEPROM writes). */
void        Switches_Tick1ms(void);

bool        Switch_IsPressed(uint8_t idx);   // debounced level
uint32_t    Switch_HeldMs(uint8_t idx);      // how long held now, 0 if released
SwitchEvent Switch_GetEvent(uint8_t idx);    // queued short/long event
void        Switch_ConsumeHold(uint8_t idx); // current press gives no more short/long events
