/*
 *  File: rn2483.c
 *  Description: Model for LoRaWAN tranceiver "Microchip RN2483"
 *  Author: Sam
 */

#include "../../../Board.h"
#include "rn2483.h"


#define RN2483_RXBUFFER_SIZE 20
#define RN2483_READ_TIMEOUT 1000
#define RN2483_BAUD_RATE 57600

static UART_Handle uart;
static UART_Params uartParams;
static int rn2483_initialised = 1;

//responses from the modem are "encapsulated" in "\r\n"

static const char rn_txbeacon[] = "mac tx uncnf 1 deadbeef\r\n";
static const char rn_txstart[] = "mac tx uncnf 1 ";
static const char rn_txstop[] = "\r\n";

static const char rn_join[] = "mac join abp\r\n";
static const char rn_reset[] = "sys reset\r\n";

int rn2483_open(){


	UART_Params_init(&uartParams);
	uartParams.writeDataMode = UART_DATA_BINARY;
	uartParams.readDataMode = UART_DATA_BINARY;
	uartParams.readReturnMode = UART_RETURN_FULL;
	uartParams.readTimeout = RN2483_READ_TIMEOUT;
	uartParams.readEcho = UART_ECHO_OFF;
	uartParams.baudRate = 57600;

	uart = UART_open(Board_UART2_COMM, &uartParams);

	if (uart == NULL) {
		return 0;
	}else{
		return 1;
	}
}


//must be called from within a task - this function will block!
int rn2483_begin(){

	if(!rn2483_open()){
		cli_printf("rn2483 open error \n",0);
		return 0;
	}else{
		cli_printf("rn2483 opened\n",0);

	}


	char rxBuffer[RN2483_RXBUFFER_SIZE];

	//reset the rn2483
	//UART_write(uart, rn_reset, sizeof(rn_reset));
	//UART_read(uart, rxBuffer, sizeof(rxBuffer));

	Task_sleep(500);

	UART_write(uart, rn_join, sizeof(rn_join));
	UART_read(uart, rxBuffer, sizeof(rxBuffer));

	Task_sleep(500);


	if(!strcmp("ok\r\n", rxBuffer)){
		return 1; //modem can now communicate with us
	}else{
		return 0;
	}

}


void rn2483_end(){


	UART_close(uart);
	uart = NULL;

	//put module to sleep
	//GPIO_write(Board_ROCKBLOCK_SLEEP, ROCKBLOCK_SLEEP);
}



int rn2483_send_receive(char * tx_buffer, int tx_size){
	char rxBuffer[RN2483_RXBUFFER_SIZE];
	char txBuffer[50];

	if(!rn2483_initialised) return 0;

	strcat(txBuffer, rn_txstart);
	strcat(txBuffer, tx_buffer);
	strcat(txBuffer, rn_txstop);

	int tx_ret = UART_write(uart, txBuffer, strlen(txBuffer));
	int rx_ret = UART_read(uart, rxBuffer, sizeof(rxBuffer));

	cli_printf("tx %d\n", tx_ret);

	if(!strcmp("ok\r\n", rxBuffer)){
		return 1; //modem can now communicate with us
	}else{
		return 0;
	}


}


