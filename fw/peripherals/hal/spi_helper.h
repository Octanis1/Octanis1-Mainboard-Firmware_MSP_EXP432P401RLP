/*
 * spi_helper.h
 *
 *  Created on: 29 Jan 2016
 *      Author: raffael
 */

#ifndef FW_PERIPHERALS_HAL_SPI_HELPER_H_
#define FW_PERIPHERALS_HAL_SPI_HELPER_H_


void spi_helper_init_handle();

uint8_t spi_helper_transfer(uint8_t nBytes, uint8_t* txBufferPointer, uint8_t* rxBufferPointer, uint8_t CS_pin);

#endif /* FW_PERIPHERALS_HAL_SPI_HELPER_H_ */
