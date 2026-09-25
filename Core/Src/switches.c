#include "switches.h"
#include "main.h"

#define SWITCH_COUNT 4
#define DEBOUNCE_MS  20       // input must be stable this long (1 ms samples)
#define MAX_QUEUED   3        // short presses remembered per switch

static volatile uint16_t s_longPressMs = 2000;
static volatile bool     s_ready       = false;

/* Per-switch state - written only in Switches_Tick1ms() (SysTick) */
static volatile bool     last_raw[SWITCH_COUNT];
static volatile uint8_t  stable_ms[SWITCH_COUNT];
static volatile bool     pressed[SWITCH_COUNT];
static volatile uint32_t press_start_ms[SWITCH_COUNT];
static volatile bool     hold_consumed[SWITCH_COUNT];   /* long fired / repeat used */
static volatile uint8_t  pending_short[SWITCH_COUNT];
static volatile bool     pending_long[SWITCH_COUNT];

/* --- Read GPIO for each switch (active-low) --- */
static bool read_pressed(uint8_t idx)
{
    switch (idx) {
        case 0: return HAL_GPIO_ReadPin(SWITCH1_GPIO_Port, SWITCH1_Pin) == GPIO_PIN_RESET; // SW1 - RED
        case 1: return HAL_GPIO_ReadPin(SWITCH2_GPIO_Port, SWITCH2_Pin) == GPIO_PIN_RESET; // SW2 - YELLOW “P”
        case 2: return HAL_GPIO_ReadPin(SWITCH3_GPIO_Port, SWITCH3_Pin) == GPIO_PIN_RESET; // SW3 - UP
        case 3: return HAL_GPIO_ReadPin(SWITCH4_GPIO_Port, SWITCH4_Pin) == GPIO_PIN_RESET; // SW4 - DOWN
        default: return false;
    }
}

/* --- Initialize switch states --- */
void Switches_Init(void)
{
    s_ready = false;
    for (int i = 0; i < SWITCH_COUNT; ++i) {
        last_raw[i]       = false;
        stable_ms[i]      = 0;
        pressed[i]        = false;
        press_start_ms[i] = 0;
        hold_consumed[i]  = false;
        pending_short[i]  = 0;
        pending_long[i]   = false;
    }
    s_ready = true;
}

void Switches_SetLongPressMs(uint16_t ms)
{
    s_longPressMs = ms;
}

void Switches_Tick1ms(void)
{
    if (!s_ready) return;
    uint32_t t = HAL_GetTick();

    for (uint8_t i = 0; i < SWITCH_COUNT; i++)
    {
        bool raw = read_pressed(i);
        if (raw != last_raw[i]) { last_raw[i] = raw; stable_ms[i] = 0; }
        else if (stable_ms[i] < 255) stable_ms[i]++;

        if (stable_ms[i] >= DEBOUNCE_MS && raw != pressed[i])
        {
            pressed[i] = raw;
            if (raw)
            {
                press_start_ms[i] = t;
                hold_consumed[i]  = false;
            }
            else if (!hold_consumed[i] && pending_short[i] < MAX_QUEUED)
            {
                pending_short[i]++;
            }
        }

        if (pressed[i] && !hold_consumed[i] &&
            (t - press_start_ms[i]) >= s_longPressMs)
        {
            hold_consumed[i] = true;
            pending_long[i]  = true;
        }
    }
}

/* --- Current pressed state (debounced) --- */
bool Switch_IsPressed(uint8_t idx)
{
    if (idx >= SWITCH_COUNT) return false;
    return pressed[idx];
}

uint32_t Switch_HeldMs(uint8_t idx)
{
    if (idx >= SWITCH_COUNT || !pressed[idx]) return 0;
    return HAL_GetTick() - press_start_ms[idx];
}

/* --- One-shot SHORT/LONG event, oldest first --- */
SwitchEvent Switch_GetEvent(uint8_t idx)
{
    if (idx >= SWITCH_COUNT) return SWITCH_EVT_NONE;

    SwitchEvent e = SWITCH_EVT_NONE;
    __disable_irq();
    if (pending_short[idx])     { pending_short[idx]--; e = SWITCH_EVT_SHORT; }
    else if (pending_long[idx]) { pending_long[idx] = false; e = SWITCH_EVT_LONG; }
    __enable_irq();
    return e;
}

void Switch_ConsumeHold(uint8_t idx)
{
    if (idx >= SWITCH_COUNT) return;
    __disable_irq();
    if (pressed[idx]) hold_consumed[idx] = true;
    __enable_irq();
}
