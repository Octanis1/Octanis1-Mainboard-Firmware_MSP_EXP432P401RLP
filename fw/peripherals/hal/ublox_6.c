#include "../../../Board.h"
#include "ublox_6.h"


static char rxBuffer[UBLOX_6_NMEABUFFER_SIZE];


static UART_Handle uart;
static UART_Params uartParams;

int ublox_6_open(){

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

		uart = UART_open(Board_UART1_GPS, &uartParams);


		if (uart == NULL) {
			return 0;
		}else{
			return 1;
		}


}

void ublox_6_close(){

	UART_close(uart);

}



char * ublox_6_read(){

	UART_read(uart, rxBuffer, sizeof(rxBuffer));
//	System_printf("%s \n", rxBuffer);
	    /* SysMin will only print to the console when you call flush or exit */
//	    System_flush();
	return rxBuffer;
}
