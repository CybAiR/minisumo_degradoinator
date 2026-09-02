#ifndef UART_H
#define UART_H

#include "main.h"

void uartWrite(const char* pText);
void uartWriteValue(const char* pName, uint32_t value);
void uartWriteQtr(uint32_t left, uint32_t right);
void uartWriteSharp(uint32_t left, uint32_t middle, uint32_t right);

#endif
