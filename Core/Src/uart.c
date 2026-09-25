#include "uart.h"
#include <string.h>
#include <stdbool.h>

extern UART_HandleTypeDef huart1;

#define UART_START_MARKER '@'
#define UART_END_MARKER   '#'

/* Received packets are queued: the web module often sends two commands
 * back to back (e.g. "@AUTO:OFF#@TIMER:ON#"); with a single buffer the
 * second one arrived while the first was still unread and was lost. */
#define UART_RX_QUEUE_LEN 4

static uint8_t  rxByte;
static char     rxBuffer[UART_RX_BUFFER_SIZE];
static char     rxQueue[UART_RX_QUEUE_LEN][UART_RX_BUFFER_SIZE];
static volatile uint8_t rxHead = 0;   /* next slot to fill (ISR)   */
static volatile uint8_t rxTail = 0;   /* next slot to read (main)  */
static uint16_t rxIndex = 0;
static volatile bool inPacket = false;

void UART_Init(void)
{
    memset(rxBuffer, 0, sizeof(rxBuffer));
    rxIndex = 0;
    rxHead = rxTail = 0;
    inPacket = false;
    HAL_UART_Receive_IT(&huart1, &rxByte, 1);
}

void UART_TransmitString(UART_HandleTypeDef *huart, const char *s)
{
    if (s && *s)
        HAL_UART_Transmit(huart, (uint8_t*)s, strlen(s), 100);
}

/* Sends "@payload#". Payloads that already carry the markers are sent
 * as they are. No fixed-size copy: the old 48-byte buffer cut long
 * replies (settings) and then transmitted past its end. */
void UART_TransmitPacket(const char *payload)
{
    if (!payload || !*payload) return;

    size_t len = strlen(payload);
    static const uint8_t start = UART_START_MARKER, end = UART_END_MARKER;

    if (payload[0] != UART_START_MARKER)
        HAL_UART_Transmit(&huart1, (uint8_t*)&start, 1, 10);
    HAL_UART_Transmit(&huart1, (uint8_t*)payload, len, 200);
    if (payload[len - 1] != UART_END_MARKER)
        HAL_UART_Transmit(&huart1, (uint8_t*)&end, 1, 10);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) return;

    uint8_t b = rxByte;

    if (b == UART_START_MARKER) {
        inPacket = true;
        rxIndex = 0;
    }
    else if (inPacket && b == UART_END_MARKER) {
        uint8_t next = (uint8_t)((rxHead + 1) % UART_RX_QUEUE_LEN);
        if (next != rxTail)                       /* drop only if queue full */
        {
            rxBuffer[rxIndex] = '\0';
            memcpy(rxQueue[rxHead], rxBuffer, rxIndex + 1);
            rxHead = next;
        }
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

    HAL_UART_Receive_IT(&huart1, &rxByte, 1);
}

bool UART_GetReceivedPacket(char *buffer, size_t buffer_size)
{
    if (rxTail == rxHead) return false;
    __disable_irq();
    strncpy(buffer, rxQueue[rxTail], buffer_size - 1);
    buffer[buffer_size - 1] = '\0';
    rxTail = (uint8_t)((rxTail + 1) % UART_RX_QUEUE_LEN);
    __enable_irq();
    return true;
}
