#include "uart.h"

#include <stdio.h>
#include <string.h>

#define UART_TIMEOUT_MS 10U

extern UART_HandleTypeDef huart2;

void uartWrite(const char* pText)
{
    HAL_UART_Transmit(&huart2, (const uint8_t*)pText, strlen(pText), UART_TIMEOUT_MS);
}

void uartWriteValue(const char* pName, uint32_t value)
{
    char buffer[32] = {0};
    int  length     = snprintf(buffer, sizeof(buffer), "%s: %lu  ", pName, (unsigned long)value);
    if (length <= 0)
        return;

    uint16_t bytes =
        length < (int)sizeof(buffer) ? (uint16_t)length : (uint16_t)(sizeof(buffer) - 1U);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, bytes, UART_TIMEOUT_MS);
}

void uartWriteQtr(uint32_t left, uint32_t right)
{
    uartWriteValue("qtr1", left);
    uartWriteValue("qtr2", right);
    uartWrite("\r\n");
}

void uartWriteSharp(uint32_t left, uint32_t middle, uint32_t right)
{
    uartWriteValue("sharp1", left);
    uartWriteValue("sharp2", middle);
    uartWriteValue("sharp3", right);
    uartWrite("\r\n");
}
