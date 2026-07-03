#include "relay.h"
#include "main.h"

#define NUM_RELAYS   3
#define RELAY_ACTIVE_STATE GPIO_PIN_SET
typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
} Relay_t;
static const Relay_t relays[NUM_RELAYS] = {
    { Relay1_GPIO_Port, Relay1_Pin },
    { Relay2_GPIO_Port, Relay2_Pin },
    { Relay3_GPIO_Port, Relay3_Pin }
};
void Relay_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    for (int i = 0; i < NUM_RELAYS; i++) {
        GPIO_InitStruct.Pin = relays[i].pin;
        HAL_GPIO_Init(relays[i].port, &GPIO_InitStruct);
        HAL_GPIO_WritePin(relays[i].port, relays[i].pin,
                          (RELAY_ACTIVE_STATE == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }
}
void Relay_Set(uint8_t relay_no, bool on)
{
    if (relay_no == 0 || relay_no > NUM_RELAYS) return;

    HAL_GPIO_WritePin(relays[relay_no - 1].port,
                      relays[relay_no - 1].pin,
                      on ? RELAY_ACTIVE_STATE :
                           (RELAY_ACTIVE_STATE == GPIO_PIN_SET ? GPIO_PIN_RESET : GPIO_PIN_SET));
}
bool Relay_Get(uint8_t relay_no)
{
    if (relay_no == 0 || relay_no > NUM_RELAYS) return false;
    GPIO_PinState s = HAL_GPIO_ReadPin(relays[relay_no - 1].port,
                                       relays[relay_no - 1].pin);
    return (s == RELAY_ACTIVE_STATE);
}
void Relay_All(bool on)
{
    for (int i = 1; i <= NUM_RELAYS; i++) {
        Relay_Set(i, on);
    }
}
