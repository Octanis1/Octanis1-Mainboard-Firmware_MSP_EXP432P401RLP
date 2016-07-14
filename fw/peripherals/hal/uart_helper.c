/*
 * uart_helper.c
 *
 *  Created on: 11 Jul 2016
 *      Author: raffael
 */

#include "uart_helper.h"

size_t uart_serial_write(void *dev, const uint8_t *data, size_t n)
{
	return UART_write(((UART_SerialDevice *)dev)->uart, data, n);
}

size_t uart_serial_read(void *dev, uint8_t *data, size_t n)
{
	return UART_read(((UART_SerialDevice *)dev)->uart, data, n);
}

int uart_serial_putc(void *dev, uint8_t c)
{
	UART_write(((UART_SerialDevice *)dev)->uart, &c, 1);
	return 0;
}

int uart_serial_getc(void *dev)
{
	uint8_t c;
	if (UART_read(((UART_SerialDevice *)dev)->uart, &c, 1) == 1) {
		return c;
	}
	return SERIAL_EOF;
}

const struct serial_function_table UART_SerialDevice_fntab = {
	uart_serial_write,
	uart_serial_read,
	uart_serial_putc,
	uart_serial_getc
};


