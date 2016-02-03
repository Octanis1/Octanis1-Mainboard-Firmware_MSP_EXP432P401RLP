/*
 * spi_helper.c
 *
 *  Created on: 29 Jan 2016
 *      Author: raffael
 */


#include "../../../Board.h"
#include "spi_helper.h"

//global variable accessible by all tasks performing spi transactions.
SPI_Handle spi_helper_handle=0;

/*Place this function in every task that needs i2c. It will know by itself if the handle has been opened already*/
void spi_helper_init_handle(){

	if(!spi_helper_handle) //only initialize for the first time.
	{
		/* Initialise I2C Bus */
		SPI_Params      params;
		SPI_Params_init(&params); /*	Defaults values are:
									transferMode = SPI_MODE_BLOCKING
									transferTimeout = SPI_WAIT_FOREVER
									transferCallbackFxn = NULL
									mode = SPI_MASTER
									bitRate = 1000000 (Hz)
									dataSize = 8 (bits)
									frameFormat = SPI_POL0_PHA0 */
		spi_helper_handle = SPI_open(Board_SPI, &params);
	}

}

uint8_t spi_helper_transfer(uint8_t nBytes, uint8_t* txBufferPointer, uint8_t* rxBufferPointer)
{
	static SPI_Transaction spiTransaction;
	spiTransaction.count = nBytes;
	spiTransaction.txBuf = txBufferPointer;
	spiTransaction.rxBuf = rxBufferPointer;
	static uint8_t ret;
	ret = SPI_transfer(spi_helper_handle, &spiTransaction);
	if (!ret) {
	   // cli_printf("Unsuccessful SPI transfer");
	}

	return ret;
}




