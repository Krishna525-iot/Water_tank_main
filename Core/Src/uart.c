#include "uart.h"
#include <string.h>
#include <stdbool.h>

extern UART_HandleTypeDef huart1;

#define UART_START_MARKER '@'
#define UART_END_MARKER   '#'

static uint8_t  rxByte;
static char     rxBuffer[UART_RX_BUFFER_SIZE];
static char     rxReadyBuffer[UART_RX_BUFFER_SIZE];
static uint16_t rxIndex = 0;
static volatile bool packetReady = false;
static volatile bool inPacket = false;

void UART_Init(void)
{
    memset(rxBuffer, 0, sizeof(rxBuffer));
    memset(rxReadyBuffer, 0, sizeof(rxReadyBuffer));
    rxIndex = 0;
    packetReady = false;
    inPacket = false;
    HAL_UART_Receive_IT(&huart1, &rxByte, 1);
}

void UART_TransmitString(UART_HandleTypeDef *huart, const char *s)
{
    if (s && *s)
        HAL_UART_Transmit(huart, (uint8_t*)s, strlen(s), 100);
}

void UART_TransmitPacket(const char *payload)
{
    static char out[48];  // small, static TX buffer
    if (!payload || !*payload) return;

    size_t len = snprintf(out, sizeof(out), "@%s#", payload);
    HAL_UART_Transmit(&huart1, (uint8_t*)out, len, 100);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) return;

    uint8_t b = rxByte;

    if (!packetReady)
    {
        if (b == UART_START_MARKER) {
            inPacket = true;
            rxIndex = 0;
        }
        else if (inPacket && b == UART_END_MARKER) {
            rxBuffer[rxIndex] = '\0';
            strncpy(rxReadyBuffer, rxBuffer, sizeof(rxReadyBuffer) - 1);
            packetReady = true;
            inPacket = false;
            rxIndex = 0;
        }
        else if (inPacket && rxIndex < (sizeof(rxBuffer) - 2)) {
            rxBuffer[rxIndex++] = b;
        }
        else if (inPacket) {
            // overflow → reset
            inPacket = false;
            rxIndex = 0;
        }
    }

    HAL_UART_Receive_IT(&huart1, &rxByte, 1);
}

bool UART_GetReceivedPacket(char *buffer, size_t buffer_size)
{
    if (!packetReady) return false;
    __disable_irq();
    strncpy(buffer, rxReadyBuffer, buffer_size - 1);
    buffer[buffer_size - 1] = '\0';
    packetReady = false;
    __enable_irq();
    return true;
}
