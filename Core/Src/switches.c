#include "switches.h"
#include "main.h"

#define SWITCH_COUNT 4
#define DEBOUNCE_MS  25       // stable, safe debounce
static uint16_t s_longPressMs = 3000;  // YOU SPECIFIED 3 seconds

/* Per-switch state */
static GPIO_PinState stable_state[SWITCH_COUNT];
static uint32_t      last_change[SWITCH_COUNT];
static GPIO_PinState last_raw[SWITCH_COUNT];
static uint32_t      press_start_ms[SWITCH_COUNT];
static bool          long_fired[SWITCH_COUNT];

static inline uint32_t now_ms(void) { return HAL_GetTick(); }

/* --- Read GPIO for each switch --- */
static GPIO_PinState read_raw(uint8_t idx)
{
    switch (idx) {
        case 0: return HAL_GPIO_ReadPin(SWITCH1_GPIO_Port, SWITCH1_Pin); // SW1 - RED
        case 1: return HAL_GPIO_ReadPin(SWITCH2_GPIO_Port, SWITCH2_Pin); // SW2 - YELLOW “P”
        case 2: return HAL_GPIO_ReadPin(SWITCH3_GPIO_Port, SWITCH3_Pin); // SW3 - UP
        case 3: return HAL_GPIO_ReadPin(SWITCH4_GPIO_Port, SWITCH4_Pin); // SW4 - DOWN
        default: return GPIO_PIN_SET;
    }
}

/* --- Initialize switch states --- */
void Switches_Init(void)
{
    for (int i = 0; i < SWITCH_COUNT; ++i) {
        stable_state[i]   = GPIO_PIN_SET;
        last_raw[i]       = GPIO_PIN_SET;
        last_change[i]    = now_ms();
        press_start_ms[i] = 0;
        long_fired[i]     = false;
    }
}

void Switches_SetLongPressMs(uint16_t ms)
{
    s_longPressMs = ms;
}

/* --- Debounce handler --- */
static void update_state(uint8_t idx)
{
    GPIO_PinState raw = read_raw(idx);
    uint32_t t = now_ms();

    if (raw != last_raw[idx]) {
        last_raw[idx] = raw;
        last_change[idx] = t;
    }

    if ((t - last_change[idx]) >= DEBOUNCE_MS)
        stable_state[idx] = raw;
}

/* --- Current pressed state (debounced) --- */
bool Switch_IsPressed(uint8_t idx)
{
    if (idx >= SWITCH_COUNT) return false;
    update_state(idx);
    return (stable_state[idx] == GPIO_PIN_RESET);
}

/* --- One-shot SHORT/LONG event generator --- */
SwitchEvent Switch_GetEvent(uint8_t idx)
{
    if (idx >= SWITCH_COUNT) return SWITCH_EVT_NONE;

    update_state(idx);
    uint32_t t = now_ms();
    bool held = (stable_state[idx] == GPIO_PIN_RESET);

    if (held)
    {
        if (press_start_ms[idx] == 0) {
            press_start_ms[idx] = t;
            long_fired[idx] = false;
        }

        if (!long_fired[idx] && (t - press_start_ms[idx] >= s_longPressMs)) {
            long_fired[idx] = true;
            return SWITCH_EVT_LONG;
        }

        return SWITCH_EVT_NONE;
    }
    else
    {
        if (press_start_ms[idx] != 0)
        {
            bool wasLong = long_fired[idx];
            press_start_ms[idx] = 0;
            long_fired[idx] = false;

            return wasLong ? SWITCH_EVT_NONE : SWITCH_EVT_SHORT;
        }
    }

    return SWITCH_EVT_NONE;
}
