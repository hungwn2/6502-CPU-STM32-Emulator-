#ifndef UART_H
#define UART_H

#include <stdint.h>

void uart_init(uint32_t pclk1_hz, uint32_t baud);
void uart_write_byte(uint8_t b);
void uart_write_string(const char *s);

#endif
