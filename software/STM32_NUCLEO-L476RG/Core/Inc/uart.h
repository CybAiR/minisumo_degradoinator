#ifndef UART_H
#define UART_H

#include "main.h"

void UART_write(const char *text);
void UART_write_value(const char *name, uint32_t value);
void UART_write_qtr(uint32_t left, uint32_t right);
void UART_write_sharp(uint32_t left, uint32_t middle, uint32_t right);

#endif
