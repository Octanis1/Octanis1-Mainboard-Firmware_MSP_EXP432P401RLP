/*
 * AS5050A.c
 *
 *  Created on: 04 Feb 2016
 *      Author: raffael
 *      based on AS5050-AB-v1.1 Adapterboard OPERATION MANUAL, 2010, ams AG
 */

#include "../../../Board.h"
#include "spi_helper.h"
#include "AS5050A.h"

/*! *****************************************************************************
* Reads out chip data via SPI interface
*
* This function is used to read out cordic value from chips supporting SPI
* interface. ***************************************************************************** */
#define SPI_CMD_READ 	0x8000  	/*!< flag indicating read attempt when using SPI interface */
#define SPI_REG_DATA 	0x7ffe	/*!< data register when using SPI */
#define SPI_REG_AGC  	0x7ff0	/*!< agc register when using SPI */
#define SPI_REG_CLRERR 	0x6700	/*!< clear error register when using SPI */

uint8_t spiCalcEvenParity(uint16_t value);

void as5050_read_data() {
	UShort tx_dat[2]; // 16-bit data buffer for SPI communication
	UShort rx_dat[2]; // 16-bit data buffer for SPI communication
//    uint16_t angle, agcreg;
//    uint16_t agc;
//    uint16_t value;
//    uint16_t alarmHi, alarmLo;

    spi_helper_transfer(sizeof(uint16_t),tx_dat, rx_dat);


/* Send READ AGC command. Received data is thrown away: this data comes from the precedent command (unknown)*/
//    tx_dat  = SPI_CMD_READ | SPI_REG_AGC;
//    tx_dat |= spiCalcEvenParity(tx_dat);
//    spi_helper_transfer(sizeof(uint16_t),(uint8_t*)&tx_dat, (uint8_t*)&rx_dat);
///* Send READ ANGLE command. Received data is the AGC value, from the precedent command */
//    tx_dat  = SPI_CMD_READ | SPI_REG_DATA;
//    tx_dat |= spiCalcEvenParity(tx_dat);
//    spi_helper_transfer(sizeof(uint16_t),(uint8_t*)&tx_dat, (uint8_t*)&rx_dat);
//    agcreg = rx_dat;
//
///* Send NOP command. Received data is the ANGLE value, from the precedent command */
//    tx_dat = 0x0000; // NOP command.
//    spi_helper_transfer(sizeof(uint16_t),(uint8_t*)&tx_dat, (uint8_t*)&rx_dat);
//    angle = rx_dat >> 2;
//
//    if (((rx_dat >> 1) & 0x1) || ((agcreg >> 1) & 0x1))
//    {
//		/* error flag set - need to reset it */
//		tx_dat  = SPI_CMD_READ | SPI_REG_CLRERR;
//		tx_dat |= spiCalcEvenParity(tx_dat);
//	    spi_helper_transfer(sizeof(uint16_t),(uint8_t*)&tx_dat, (uint8_t*)&rx_dat);
//    }
//    else
//    {
//		agc = (agcreg >> 2) & 0x3f; 		// AGC value (0..63)
//		value = (rx_dat >> 2) & 0x3fff;		// Angle value (0..4095 for AS5055)
//		angle = (value * 360) / 4095; 	// Angle value in degree (0..359.9°)
//		alarmLo = (rx_dat >> 14) & 0x1;
//		alarmHi = (rx_dat >> 15) & 0x1;
//    }
}

/*! *****************************************************************************
* Calculate even parity of a 16 bit unsigned integer
*
* This function is used by the SPI interface to calculate the even parity
* of the data which will be sent via SPI to the encoder.
*
* \param[in] value : 16 bit unsigned integer whose parity shall be calculated *
* \return : Even parity
* ***************************************************************************** */
uint8_t spiCalcEvenParity(uint16_t value) {
	uint8_t cnt = 0;
	uint8_t i;

    for (i = 0; i < 16; i++)
    {
        if (value & 0x1)
        {
        		cnt++;
        }
        value = value >> 1;
    }
    return (cnt & 0x1);
}







