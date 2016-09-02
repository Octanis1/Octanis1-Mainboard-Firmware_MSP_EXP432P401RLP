#include "../../../Board.h"
#include "ublox_6.h"

static char ublox_rxBuffer[UBLOX_6_NMEABUFFER_SIZE];

static UART_Handle ublox_uart;

void ublox_6_open(){
	static UART_Params uartParams;

	/* Create a UART with data processing off. */
	UART_Params_init(&uartParams);
	uartParams.writeDataMode = UART_DATA_TEXT;
	uartParams.readDataMode = UART_DATA_TEXT;
	uartParams.readReturnMode = UART_RETURN_NEWLINE; //one NMEA frame per read
	uartParams.readEcho = UART_ECHO_OFF;
	uartParams.baudRate = 9600;
	uartParams.readMode = UART_MODE_BLOCKING;
	//uartParams.readTimeout = 10;
	uartParams.dataLength = UART_LEN_8;

	//Correct port for the mainboard
	ublox_uart = UART_open(Board_UART1_GPS, &uartParams);

	if (ublox_uart == NULL)
		System_abort("Error opening the GPS UART");
}

void ublox_6_close(){

	UART_close(ublox_uart);

}



char* ublox_6_read(){

//	int i=0;
	UART_read(ublox_uart, ublox_rxBuffer, sizeof(ublox_rxBuffer));

//	for(i=0;ublox_rxBuffer[i]!='\0';i++){
//		System_printf("%c \n", ublox_rxBuffer[i]);
	    	/* SysMin will only print to the console when you call flush or exit */
//	    System_flush();
//	}
	return ublox_rxBuffer;
}
