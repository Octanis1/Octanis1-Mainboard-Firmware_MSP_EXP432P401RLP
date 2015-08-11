/*
 *  File: rockblock.c
 *  Description: Model for Iridium Satellite Modem "Rockblock"
 *  Author: Sam
 */

#include "../../../Board.h"

#include "rockblock.h"

#define ROCKBLOCK_WAKE 1
#define ROCKBLOCK_SLEEP 0

#define ROCKBLOCK_RXBUFFER_SIZE 100
#define ROCKBLOCK_TXBUFFER_SIZE 100

static char rxBuffer[ROCKBLOCK_RXBUFFER_SIZE];
static char txBuffer[ROCKBLOCK_TXBUFFER_SIZE];

static UART_Handle uart;
static UART_Params uartParams;

int rockblock_open(){

	System_printf("waking up rb");
	System_flush();

	//wake rockblock from sleep
    GPIO_write(Board_ROCKBLOCK_SLEEP, ROCKBLOCK_WAKE);


	/* Create a UART with data processing off. */
	UART_Params_init(&uartParams);
	uartParams.writeDataMode = UART_DATA_BINARY;
	uartParams.readDataMode = UART_DATA_BINARY;
	uartParams.readReturnMode = UART_RETURN_NEWLINE;
	uartParams.readEcho = UART_ECHO_OFF;
	uartParams.baudRate = 9600;

	uart = UART_open(Board_UART2_COMM, &uartParams);

	if (uart == NULL) {
		return 0;
	}else{
		return 1;
	}
}

int rockblock_begin(){
	//clear the receive buffer
	memset(&rxBuffer[0], 0, sizeof(rxBuffer));


	UART_write(uart, "AT\r", 3);

	UART_read(uart, rxBuffer, sizeof(rxBuffer));

	cli_printf(rxBuffer, 0);

	return 1;

}


void rockblock_close(){
	UART_close(uart);

    GPIO_write(Board_ROCKBLOCK_SLEEP, ROCKBLOCK_SLEEP);

}


int rockblock_get_sleep_status(){
	return GPIO_read(Board_ROCKBLOCK_SLEEP);
}

int rockblock_get_net_availability(){
	return GPIO_read(Board_ROCKBLOCK_NET);

}

