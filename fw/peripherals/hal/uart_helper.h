/*
 * uart_helper.h
 *
 *  Created on: 11 Jul 2016
 *      Author: raffael
 */

#ifndef FW_PERIPHERALS_HAL_UART_HELPER_H_
#define FW_PERIPHERALS_HAL_UART_HELPER_H_

#include <serial.h>
#include <stdint.h>
#include "../../Board.h"

typedef struct {
    const struct serial_function_table *fntab;
    UART_Handle uart;
} UART_SerialDevice;

size_t uart_serial_write(void *dev, const uint8_t *data, size_t n);

size_t uart_serial_read(void *dev, uint8_t *data, size_t n);

int uart_serial_putc(void *dev, uint8_t c);

int uart_serial_getc(void *dev);

extern const struct serial_function_table UART_SerialDevice_fntab;


#endif /* FW_PERIPHERALS_HAL_UART_HELPER_H_ */
