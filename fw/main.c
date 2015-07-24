/*
 * Copyright (c) 2014, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>

/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/I2C.h>

/* Example/Board Header files */
#include "../Board.h"

/* External libraries */
#include "lib/minmea/minmea.h"

#include <stdint.h>
#include <string.h>
#include <ctype.h>

/*
 *  ======== echoFxn ========
 *  Task for this function is created statically. See the project's .cfg file.
 */
Void echoFxn(UArg arg0, UArg arg1)
{
		System_printf("attempting an I2C conn \n");
		System_flush();
		I2C_Handle i2c;


		UInt peripheralNum = 0;     /* Such as I2C0 */
		I2C_Params i2cParams;
		I2C_Params_init(&i2cParams);
		i2cParams.transferMode = I2C_MODE_BLOCKING;
		i2cParams.transferCallbackFxn =	NULL;

		i2c = I2C_open(peripheralNum, &i2cParams);
		if (i2c == NULL) {
		    /* Error opening I2C */
		}

	    I2C_Transaction   i2cTransaction;
	    uint8_t             writeBuffer[1];
	    uint8_t             readBuffer[1];
	    Bool              transferOK;

	    writeBuffer[0] = 0x13;
	    readBuffer[0] = 0;


	    i2cTransaction.slaveAddress = 0x29;     /* 7-bit peripheral slave address */
	    i2cTransaction.writeBuf = writeBuffer;  /* Buffer to be written */
	    i2cTransaction.writeCount = 1;          /* Number of bytes to be written */
	    i2cTransaction.readBuf = readBuffer;          /* Buffer to be read */
	    i2cTransaction.readCount = 1;           /* Number of bytes to be read */
	    transferOK = I2C_transfer(i2c, &i2cTransaction); /* Perform I2C transfer */

	    if (!transferOK) {
	        /* I2C bus fault */
	    	  System_printf("I2C error");
	    	  System_flush();
	    }else{
	    	System_printf("I2C good");
	    		    	  System_flush();
	    }

        System_printf("going into while true");
        System_flush();

	    while (true) {

	        GPIO_write(Board_LED2, Board_LED_ON);
	        Task_sleep(100);
	        GPIO_write(Board_LED2, Board_LED_OFF);

	    }
}

/*
 *  ======== ledFxn ========
 *  Task for this function is also created statically. See the project's .cfg file.
 */
Void ledFxn(UArg arg0, UArg arg1){

    System_printf("LED task started");
    System_flush();

	while (true) {
		GPIO_write(Board_LED0, Board_LED_ON);
		Task_sleep(300);
		GPIO_write(Board_LED0, Board_LED_OFF);
		Task_sleep(300);
	}

}

Void gsmFxn(UArg arg0, UArg arg1){
    GPIO_write(Board_LED2, Board_LED_ON);

	System_printf("entering GSM task and wait \n");
	System_flush();

	int ret;
	UART_Handle handle;
	UART_Params params;

	UART_Params_init(&params);
	params.baudRate = 115200;
	params.writeDataMode = UART_DATA_BINARY;
	params.readDataMode = UART_DATA_BINARY;
	params.readReturnMode = UART_RETURN_FULL;
	params.readEcho = UART_ECHO_OFF;
	params.readMode = UART_MODE_BLOCKING;
	params.dataLength = UART_LEN_8;
	params.readTimeout = 8000;

	handle = UART_open(Board_UART2, &params);
	if (!handle) {
		System_printf("UART did not open");
	}else{
		System_printf("UART opened!");
	}


	const char txBuffer[] = "AT+CBC\r";
	char rxBuffer[40];


	UART_write(handle, txBuffer, sizeof(txBuffer));

	ret = UART_readPolling(handle,rxBuffer,sizeof(rxBuffer));
		//check buffer
		System_printf("yes! %d", ret);


	System_printf("\n");
	System_flush();


    GPIO_write(Board_LED2, Board_LED_OFF);

	Task_setPri(task3, -1); //sets task to be inactive

}


Void gpsFxn(UArg arg0, UArg arg1){

	System_printf("entering GPS task \n");
	System_flush();

	char rxBuffer[300];
	int ret;

	UART_Handle uart;
	UART_Params uartParams;

	/* Create a UART with data processing off. */
	UART_Params_init(&uartParams);
	uartParams.writeDataMode = UART_DATA_TEXT;
	uartParams.readDataMode = UART_DATA_BINARY;
	uartParams.readReturnMode = UART_RETURN_FULL; //one NMEA frame per read
	uartParams.readEcho = UART_ECHO_OFF;
	uartParams.baudRate = 115200;
	uartParams.readMode = UART_MODE_BLOCKING;
	//uartParams.readTimeout = 10;
	uartParams.dataLength = UART_LEN_8;


	int j = 0;


	/* Loop forever echoing */
	while (1) {
		uart = UART_open(Board_UART1, &uartParams); //P3.2 is RX on Launchpad

		if (uart == NULL) {
			System_printf("UART error \n");
			System_flush();
		}

		ret = UART_read(uart, rxBuffer, sizeof(rxBuffer));

		char * nmeaframes = strtok(rxBuffer, "\n");

		while (nmeaframes != NULL){

			int minmea_sentence = minmea_sentence_id(nmeaframes, false);
			nmeaframes = strtok(NULL, "\n");

					switch (minmea_sentence) {

						case MINMEA_SENTENCE_GGA: {

						}break;


					      case MINMEA_SENTENCE_RMC: {

					      }break;

			            case MINMEA_INVALID: {

			             } break;

			            default: {

			             } break;
					}

		}


		/*watch stack*/
		Task_Stat statbuf;               /* declare buffer */
		Task_stat(Task_self(), &statbuf); /* call func to get status */
		if (statbuf.used > (statbuf.stackSize * 9 / 10)) {
		   System_printf("Over 90% of task's stack is in use. => %d B \n",statbuf.used);
		   System_flush();

		}

		UART_close(uart);
		Task_setPri(task2, -1); //sets task to be inactive

	}


}
/*
 *  ======== main ========
 */
int main(void)
{
    /* Call board init functions */
    Board_initGeneral();
    Board_initGPIO();
    Board_initUART();
    Board_initI2C();

    /* Turn on user LED */
    GPIO_write(Board_LED0, Board_LED_OFF);

    GPIO_write(Board_LED0, Board_LED_ON);



    /* Start BIOS */
    BIOS_start();

    return (0);
}
