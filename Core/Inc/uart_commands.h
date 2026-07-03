#pragma once
#include "main.h"
#include <stdbool.h>

void UART_HandleCommand(const char *packet);
void UART_SendStatusPacket(void);
void UART_SendDryAlert(void);
void UART_StatusTask(void);
