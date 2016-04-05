/*
 *  File: hm10.c
 *  Description: BLE modem driver
 *  Author: Sam Sulaimanov
 */

#include "../../../Board.h"

#include "hm10.h"

#define HM10_BAUD_RATE 9600
#define HM10_READ_TIMEOUT 3000
#define HM10_RXBUFFER_SIZE 100

static const char hm10_at[] = "AT"; //does not require return characters
static const char hm10_at_name[] = "AT+NAMEoctanisMars";


static UART_Handle uart;
static UART_Params uartParams;
static int hm10_initialised = 0;
static int hm10_locked = 0;


int hm10_open(){

	UART_Params_init(&uartParams);
	uartParams.writeDataMode = UART_DATA_BINARY;
	uartParams.readDataMode = UART_DATA_BINARY;
	uartParams.readReturnMode = UART_RETURN_FULL;
	uartParams.readTimeout = HM10_READ_TIMEOUT;
	uartParams.readEcho = UART_ECHO_OFF;
	uartParams.baudRate = HM10_BAUD_RATE;

	uart = UART_open(Board_UART2_COMM, &uartParams);

	if (uart == NULL) {

		//debug
		GPIO_toggle(Board_LED_GREEN);
		Task_sleep(300);
		GPIO_toggle(Board_LED_GREEN);

		return 0;
	}else{
		return 1;
	}
}


//must be called from within a task - this function will block!
//returns 1 if modem responds with OK
int hm10_begin(){
	char rxBuffer[HM10_RXBUFFER_SIZE];
	memset(&rxBuffer, 0, sizeof(rxBuffer));

	if(hm10_open()){
		UART_write(uart, hm10_at_echo_off, sizeof(hm10_at_echo_off));
		UART_read(uart, rxBuffer, sizeof(rxBuffer));
		Task_sleep(500);

		cli_printf("%s", rxBuffer);
		Task_sleep(500);

		if(!strcmp("OK", rxBuffer)){
			hm10_initialised = 1;
			return 1; //modem can now communicate with us
		}else{
			return 0;
		}
	}else{
		return 0;
	}
}

void hm10_end(){
	UART_close(uart);
	uart = NULL;
}



