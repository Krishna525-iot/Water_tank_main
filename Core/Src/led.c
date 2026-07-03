#include "led.h"
#include <string.h>
static GPIO_TypeDef* LED_PORTS[LED_COLOR_COUNT] = {
    [LED_COLOR_GREEN]  = LED1_GPIO_Port,
    [LED_COLOR_RED]    = LED2_GPIO_Port,
    [LED_COLOR_BLUE]   = LED3_GPIO_Port,
    [LED_COLOR_PURPLE] = LED4_GPIO_Port,
};
static uint16_t LED_PINS[LED_COLOR_COUNT] = {
    [LED_COLOR_GREEN]  = LED1_Pin,
    [LED_COLOR_RED]    = LED2_Pin,
    [LED_COLOR_BLUE]   = LED3_Pin,
    [LED_COLOR_PURPLE] = LED4_Pin,
};
typedef struct {
    LedMode   mode;
    uint16_t  period_ms;
} LedIntent;
static LedIntent s_intent[LED_COLOR_COUNT];
static uint8_t   s_activeBlink[LED_COLOR_COUNT];
static uint32_t  s_nextToggleAt[LED_COLOR_COUNT];
static inline uint32_t now_ms(void) { return HAL_GetTick(); }
static void led_write(LedColor c, GPIO_PinState st) {
    HAL_GPIO_WritePin(LED_PORTS[c], LED_PINS[c], st);
}
static void led_on(LedColor c)  { led_write(c, GPIO_PIN_SET); }
static void led_off(LedColor c) { led_write(c, GPIO_PIN_RESET); }
void LED_Init(void)
{
    memset(s_intent, 0, sizeof(s_intent));
    memset(s_activeBlink, 0, sizeof(s_activeBlink));
    memset(s_nextToggleAt, 0, sizeof(s_nextToggleAt));
    for (int i = 0; i < LED_COLOR_COUNT; ++i) {
        led_off((LedColor)i);
        s_intent[i].mode = LED_MODE_OFF;
        s_intent[i].period_ms = 0;
    }
}
void LED_Task(void)
{
    uint32_t t = now_ms();
    for (int i = 0; i < LED_COLOR_COUNT; ++i) {
        switch (s_intent[i].mode) {
        case LED_MODE_OFF:
            s_activeBlink[i] = 0;
            led_off((LedColor)i);
            break;
        case LED_MODE_STEADY:
            s_activeBlink[i] = 1;
            led_on((LedColor)i);
            break;
        case LED_MODE_BLINK:
        default:
            if (s_intent[i].period_ms == 0) s_intent[i].period_ms = 500;
            if ((int32_t)(s_nextToggleAt[i] - t) <= 0) {
                s_activeBlink[i] = !s_activeBlink[i];
                if (s_activeBlink[i]) led_on((LedColor)i);
                else                  led_off((LedColor)i);
                s_nextToggleAt[i] = t + s_intent[i].period_ms;
            }
            break;
        }
    }
}
void LED_ClearAllIntents(void)
{
    for (int i = 0; i < LED_COLOR_COUNT; ++i) {
        s_intent[i].mode = LED_MODE_OFF;
        s_intent[i].period_ms = 0;
    }
}
void LED_SetIntent(LedColor color, LedMode mode, uint16_t period_ms)
{
    if ((int)color < 0 || color >= LED_COLOR_COUNT) return;
    s_intent[color].mode = mode;
    s_intent[color].period_ms = period_ms;
}
void LED_ApplyIntents(void) {}
void LED_All_Off(void)
{
    for (int i = 0; i < LED_COLOR_COUNT; ++i) {
        s_intent[i].mode = LED_MODE_OFF;
        s_intent[i].period_ms = 0;
        led_off((LedColor)i);
    }
}
