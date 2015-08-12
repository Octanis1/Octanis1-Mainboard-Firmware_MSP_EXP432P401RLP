/*
 *  File: rockblock.c
 *  Description: Model for Iridium Satellite Modem "Rockblock"
 *  Author: Sam
 */

#include "../../../Board.h"
#include "rockblock.h"

#define ROCKBLOCK_WAKE 1
#define ROCKBLOCK_SLEEP 0
#define ROCKBLOCK_RXBUFFER_SIZE 20

static UART_Handle uart;
static UART_Params uartParams;

/* Iridium 9602 Modem AT commands */
static const char rockblock_at[] = "AT\r";
static const char rockblock_at_csq[] = "AT+CSQ\r";


int rockblock_open(){

	//wake rockblock from sleep
    GPIO_write(Board_ROCKBLOCK_SLEEP, ROCKBLOCK_WAKE);
	cli_printf("RB wake\n",0);

	//these uart configs are good for writing to and reading from the RB module
	//the RB must first be configured to work at 9600baud and local echo off
	UART_Params_init(&uartParams);
	uartParams.writeDataMode = UART_DATA_BINARY;
	uartParams.readDataMode = UART_DATA_BINARY;
	uartParams.readReturnMode = UART_RETURN_FULL;
	uartParams.readTimeout = 10000;
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

	char rxBuffer[ROCKBLOCK_RXBUFFER_SIZE];

	UART_write(uart, rockblock_at, sizeof(rockblock_at));
	UART_read(uart, rxBuffer, sizeof(rxBuffer));

	if(!strcmp("\r\nOK\r\n", rxBuffer)){
		return 1; //modem can communicate fine via AT commands
	}else{
		return 0;
	}

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

