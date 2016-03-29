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
#define SPI_CMD_NOP		0x0000
#define SPI_CMD_READ 	0x8000  /*!< flag indicating read attempt when using SPI interface */
#define SPI_REG_DATA 	0x7ffe	/*!< data register when using SPI */
#define SPI_REG_AGC  	0x7ff0	/*!< agc register when using SPI */
#define SPI_REG_CLRERR 	0x6700	/*!< clear error register when using SPI */
#define SPI_MASTER_RESET 0x33A5

uint8_t spiCalcEvenParity(uint16_t value);

static uint16_t as5050_command(uint16_t tx_dat)
{
    uint8_t tx[2], rx[2];
    tx_dat |= spiCalcEvenParity(tx_dat);
    tx[0] = (tx_dat>>8) & 0xff;
    tx[1] = tx_dat & 0xff;
    spi_helper_transfer(sizeof(tx), tx, rx, Board_M1_ANGLE_ENCODER_CS);
    return (uint16_t)rx[1] | ((uint16_t)rx[0]<<8);
}

bool as5050_read_data(uint16_t* angle) {
	uint16_t tx_dat; // 16-bit data buffer for SPI communication
	uint16_t rx_dat; // 16-bit data buffer for SPI communication
    uint16_t agcreg;
    uint16_t agc;
    uint16_t value;
    uint16_t alarmHi, alarmLo;

/* Send READ AGC command. Received data is thrown away: this data comes from the precedent command (unknown)*/
    as5050_command(SPI_CMD_READ | SPI_REG_AGC);

/* Send READ ANGLE command. Received data is the AGC value, from the precedent command */
    agcreg = as5050_command(SPI_CMD_READ | SPI_REG_DATA);

    Task_sleep(10); // wait till the result is valid.
/* Send NOP command. Received data is the ANGLE value, from the precedent command */
    rx_dat = as5050_command(SPI_CMD_NOP);

    if (((rx_dat >> 1) & 0x1) || ((agcreg >> 1) & 0x1))
    {
		/* error flag set - need to reset it */
		as5050_command(SPI_CMD_READ | SPI_REG_CLRERR);
		return false;
    }
    else if((rx_dat & 0xC000) == 0xC000) //system error occured... perform master reset.
    {
		as5050_command(SPI_MASTER_RESET);
		return false;
    }
    else
    {
		agc = (agcreg >> 2) & 0x3f; 		// AGC value (0..63)
		value = (rx_dat >> 2) & 0x03ff;		// Angle value (0..4095 for AS5055)
		(*angle) = ((uint64_t)value * 360) / 0x03ff; 	// Angle value in degree (0..359.9°)
		alarmLo = (rx_dat >> 14) & 0x1;
		alarmHi = (rx_dat >> 15) & 0x1;
    }
	return true;
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







