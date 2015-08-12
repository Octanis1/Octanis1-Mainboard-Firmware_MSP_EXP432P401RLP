/*
 *  File: rockblock.c
 *  Description: Model for Iridium Satellite Modem "Rockblock"
 *  Author: Sam
 */

#include "../../../Board.h"
#include "rockblock.h"

#define ROCKBLOCK_WAKE 1
#define ROCKBLOCK_WAKE_WAITTICKS 600
#define ROCKBLOCK_SLEEP 0
#define ROCKBLOCK_RXBUFFER_SIZE 20
#define ROCKBLOCK_READ_TIMEOUT 10000
#define ROCKBLOCK_BAUD_RATE 9600

static UART_Handle uart;
static UART_Params uartParams;

/* Iridium 9602 Modem AT commands */
//responses from the modem are "encapsulated" in "\r\n"
static const char rockblock_at[] = "AT\r";
static const char rockblock_at_echo_off[] = "ATE0\r";
static const char rockblock_at_csq[] = "AT+CSQ\r";


int rockblock_open(){

	//wake rockblock from sleep
    GPIO_write(Board_ROCKBLOCK_SLEEP, ROCKBLOCK_WAKE);

    //it takes 500ms for the rockblock to wake up from its beauty sleep
    Task_sleep(ROCKBLOCK_WAKE_WAITTICKS);


	//these uart configs are good for writing to and reading from the RB module
	//the RB must first be configured to work at 9600baud and local echo off
	UART_Params_init(&uartParams);
	uartParams.writeDataMode = UART_DATA_BINARY;
	uartParams.readDataMode = UART_DATA_BINARY;
	uartParams.readReturnMode = UART_RETURN_FULL;
	uartParams.readTimeout = ROCKBLOCK_READ_TIMEOUT;
	uartParams.readEcho = UART_ECHO_OFF;
	uartParams.baudRate = ROCKBLOCK_BAUD_RATE;

	uart = UART_open(Board_UART2_COMM, &uartParams);

	if (uart == NULL) {
		return 0;
	}else{
		return 1;
	}
}


//must be called from within a task - this function will block!
int rockblock_begin(){

	char rxBuffer[ROCKBLOCK_RXBUFFER_SIZE];

	//set local echo off first
	UART_write(uart, rockblock_at_echo_off, sizeof(rockblock_at_echo_off));
	//give the RB some time to respond
	UART_read(uart, rxBuffer, sizeof(rxBuffer));
	//clear the buffer
	memset(rxBuffer, 0, sizeof(rxBuffer));

	//send an AT for confirmation that local echo has been turned off
	UART_write(uart, rockblock_at, sizeof(rockblock_at));
	UART_read(uart, rxBuffer, sizeof(rxBuffer));


	if(!strcmp("\r\nOK\r\n", rxBuffer)){
		return 1; //modem can now communicate with us
	}else{
		return 0;
	}

}



void rockblock_close(){
	UART_close(uart);

    GPIO_write(Board_ROCKBLOCK_SLEEP, ROCKBLOCK_SLEEP);
}


//returns 0 (no signal) to 5 (best). iridium recommends at least a 2 before transmit
// 1 works in some conditions
int rockblock_get_signal_quality(){
	char rxBuffer[ROCKBLOCK_RXBUFFER_SIZE];
	int csq_val = 0;

	//clear the buffer
	memset(rxBuffer, 0, sizeof(rxBuffer));

	UART_write(uart, rockblock_at_csq, sizeof(rockblock_at_csq));
	int retbuf = UART_read(uart, rxBuffer, sizeof(rxBuffer));


	//signal quality value is at index 7
	csq_val = system_chartoint((char)rxBuffer[7]);
	return csq_val;
}

int rockblock_get_sleep_status(){
	return GPIO_read(Board_ROCKBLOCK_SLEEP);
}



int rockblock_get_net_availability(){
	return GPIO_read(Board_ROCKBLOCK_NET);

}

