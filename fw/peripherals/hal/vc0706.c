/*
 *  File: vc0706.c
 *  Description: UART Camera driver
 *  Author: Sam Sulaimanov
 */

#include "../../../Board.h"
#include "../../lib/printf.h"

#include "vc0706.h"
#include "sim800.h"


#define VC0706_BAUD_RATE 115200
#define VC0706_READ_TIMEOUT 1500
#define VC0706_RXBUFFER_SIZE 5000


static UART_Handle uart;
static UART_Params uartParams;
char rxBuffer[VC0706_RXBUFFER_SIZE];


int vc0706_open(){

	UART_Params_init(&uartParams);
	uartParams.writeDataMode = UART_DATA_BINARY;
	uartParams.readDataMode = UART_DATA_BINARY;
	uartParams.readReturnMode = UART_RETURN_FULL;
	uartParams.readTimeout = VC0706_READ_TIMEOUT;
	uartParams.readEcho = UART_ECHO_OFF;
	uartParams.baudRate = VC0706_BAUD_RATE;

	uart = UART_open(Board_UART3_LORACOMM, &uartParams);

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
int vc0706_begin(){

	if(vc0706_open()){
		UART_read(uart, rxBuffer, sizeof(rxBuffer));
		memset(&rxBuffer, 0, sizeof(rxBuffer));

		cli_printf("vc0706 cam on",0);

		static char vc0706_reset[4] = {0x56, 0x00, 0x26, 0x00}; //works!
		static char vc0706_takepic[5] = {0x56, 0x00, 0x36, 0x01, 0x00};
		static char vc0706_getfilesize[4] = {0x56, 0x00, 0x34, 0x00};
		static char vc0706_downloadpic[16] = {0x56, 0x00, 0x32, 0x0C, 0x00, 0x0A,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x13, 0x88, 0x00, 0x0A};

		//get fw version
		UART_write(uart, vc0706_reset, sizeof(vc0706_reset));
		UART_read(uart, rxBuffer, sizeof(rxBuffer));
		memset(&rxBuffer, 0, sizeof(rxBuffer));
		Task_sleep(500);

		//take pic
		UART_write(uart, vc0706_takepic, sizeof(vc0706_takepic));
		UART_read(uart, rxBuffer, sizeof(rxBuffer));
		memset(&rxBuffer, 0, sizeof(rxBuffer));
		Task_sleep(500);

		//get size
		UART_write(uart, vc0706_getfilesize, sizeof(vc0706_getfilesize));
		UART_read(uart, rxBuffer, sizeof(rxBuffer));
		Task_sleep(500);

		//download pic
		UART_write(uart, vc0706_downloadpic, sizeof(vc0706_downloadpic));
		UART_read(uart, rxBuffer, sizeof(rxBuffer));
		Task_sleep(5000);


		return 1;
	}else{
		return 0;
	}
}

void vc0706_gprs_upload_jpeg(){
	sim800_send_http(rxBuffer, sizeof(rxBuffer), MIME_OCTET_STREAM);
}

void vc0706_end(){
	UART_close(uart);
	uart = NULL;
}


