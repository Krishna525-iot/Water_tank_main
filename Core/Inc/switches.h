#pragma once
#include <stdbool.h>
#include <stdint.h>

/* Events reported per switch */
typedef enum {
    SWITCH_EVT_NONE = 0,
    SWITCH_EVT_SHORT,   // released before long threshold
    SWITCH_EVT_LONG     // long threshold crossed (fires once while held)
} SwitchEvent;

/* Optional: change long press threshold (default 700 ms) */
void     Switches_SetLongPressMs(uint16_t ms);

void Switches_Init(void);
/* Polling API (call each loop or from a 5–10ms tick) */
bool        Switch_IsPressed(uint8_t idx);   // debounced level (active-low)
bool        Switch_WasPressed(uint8_t idx);  // edge (kept for backward-compat)
SwitchEvent Switch_GetEvent(uint8_t idx);    // NEW: short/long event
