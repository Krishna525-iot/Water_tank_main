#ifndef RELAY_H
#define RELAY_H

#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

void Relay_Init(void);
void Relay_Set(uint8_t relay_no, bool on);
bool Relay_Get(uint8_t relay_no);
void Relay_All(bool on);

#endif /* RELAY_H */
