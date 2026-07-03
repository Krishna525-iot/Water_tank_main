#ifndef __UART_H
#define __UART_H

#include "main.h"
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#define UART_RX_BUFFER_SIZE 128
extern UART_HandleTypeDef huart1;
void UART_Init(void);
bool UART_GetReceivedPacket(char *buffer, size_t buffer_size);
void UART_TransmitString(UART_HandleTypeDef *huart, const char *str);
void UART_TransmitByte(UART_HandleTypeDef *huart, uint8_t byte);
bool UART_GetReceivedPacket(char *buffer, size_t buffer_size);

#endif
