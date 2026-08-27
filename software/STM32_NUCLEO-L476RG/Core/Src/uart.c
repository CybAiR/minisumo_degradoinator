#include "uart.h"

#include <stdio.h>
#include <string.h>

#define UART_TIMEOUT_MS 10U

extern UART_HandleTypeDef huart2;


void UART_write(const char *text)
{
  HAL_UART_Transmit(&huart2, (const uint8_t *)text, strlen(text), UART_TIMEOUT_MS);
}

void UART_write_value(const char *name, uint32_t value)
{
  char buffer[32];
  int length = snprintf(buffer, sizeof(buffer), "%s: %lu  ", name, (unsigned long)value);
  if (length <= 0)
    return;

  uint16_t bytes = length < (int)sizeof(buffer) ? (uint16_t)length : (uint16_t)(sizeof(buffer) - 1U);
  HAL_UART_Transmit(&huart2, (uint8_t *)buffer, bytes, UART_TIMEOUT_MS);
}

void UART_write_qtr(uint32_t left, uint32_t right)
{
  UART_write_value("qtr1", left);
  UART_write_value("qtr2", right);
  UART_write("\r\n");
}

void UART_write_sharp(uint32_t left, uint32_t middle, uint32_t right)
{
  UART_write_value("sharp1", left);
  UART_write_value("sharp2", middle);
  UART_write_value("sharp3", right);
  UART_write("\r\n");
}
