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

/*Place this function in every task that needs spi. It will know by itself if the handle has been opened already*/
void spi_helper_init_handle(){

	/*************** For strut encoders ************************/

//	if(!spi_helper_handle) //only initialize for the first time.
//	{
//		/* Initialise SPI Bus */
//		SPI_Params      params;
//		SPI_Params_init(&params); /*	Defaults values are:
//									transferMode = SPI_MODE_BLOCKING
//									(transferTimeout = SPI_WAIT_FOREVER), overwritten
//									transferCallbackFxn = NULL
//									mode = SPI_MASTER
//									bitRate = 1000000 (Hz)
//									dataSize = 8 (bits); dataSize can range from 4 to 8 bits
//									frameFormat = SPI_POL0_PHA0 */
//		params.transferTimeout = 100;
//		params.frameFormat = SPI_POL0_PHA1;
//		spi_helper_handle = SPI_open(Board_SPI, &params);
//	}
//
//	if(spi_helper_handle == NULL)
//	{
//		GPIO_toggle(Board_LED_GREEN);
//	}


	if(!spi_helper_handle) //only initialize for the first time.
	{
		/* Initialise SPI Bus */
		SPI_Params      params;
		SPI_Params_init(&params); /*	Defaults values are:
									transferMode = SPI_MODE_BLOCKING
									(transferTimeout = SPI_WAIT_FOREVER), overwritten
									transferCallbackFxn = NULL
									mode = SPI_MASTER
									bitRate = 1000000 (Hz)
									dataSize = 8 (bits); dataSize can range from 4 to 8 bits
									frameFormat = SPI_POL0_PHA0 */
//		params.transferTimeout = 100;
		params.frameFormat = SPI_POL1_PHA1; // for flash
		spi_helper_handle = SPI_open(Board_SPI, &params);
	}
}

uint8_t spi_helper_transfer(uint8_t nBytes, uint8_t* txBufferPointer, uint8_t* rxBufferPointer, uint8_t CS_pin)
{
	static SPI_Transaction spiTransaction;
	spiTransaction.count = nBytes;
	spiTransaction.txBuf = txBufferPointer;
	spiTransaction.rxBuf = rxBufferPointer;
	static uint8_t ret;
//	GPIO_write(CS_pin, 0);
	ret = SPI_transfer(spi_helper_handle, &spiTransaction);
//	GPIO_write(CS_pin, 1);

	if (!ret) {
//	   serial_printf(cli_stdout, "Unsuccessful SPI transfer");
		return 1;
	} else {
		return 0;
	}
}




