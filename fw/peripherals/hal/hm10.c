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
static const char hm10_at_name[] = "AT+NAMEmars";
static const char hm10_at_clear[] = "AT+CLEAR"; //clear last connected device
static const char hm10_at_wakestring[] = "I am iron man,I am iron man,I am iron man,I am iron man, I am iron man, I am iron man I am iron I am iron man, I am iron man.";
static const char hm10_at_imme1[] = "AT+IMME1"; //When module is powered on, only respond the AT Command, don’t do anything.
static const char hm10_at_start[] = "AT+START"; //start connecting to devices
static const char hm10_at_pwrm1[] = "AT+PWRM1"; //doesnt go to sleep in this mode
static const char hm10_at_pwrm0[] = "AT+PWRM0"; //sleep mode

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
		Task_sleep(1000);

		UART_write(uart, hm10_at_wakestring, strlen(hm10_at_wakestring));
		UART_read(uart, rxBuffer, sizeof(rxBuffer));
		cli_printf("%s\n", rxBuffer);
		memset(&rxBuffer, 0, sizeof(rxBuffer));

		UART_write(uart, hm10_at_pwrm1, strlen(hm10_at_pwrm1));
		UART_read(uart, rxBuffer, sizeof(rxBuffer));
		cli_printf("%s\n", rxBuffer);
		memset(&rxBuffer, 0, sizeof(rxBuffer));
		Task_sleep(1000);

		UART_write(uart, hm10_at_clear, strlen(hm10_at_clear));
		UART_read(uart, rxBuffer, sizeof(rxBuffer));
		cli_printf("%s\n", rxBuffer);
		memset(&rxBuffer, 0, sizeof(rxBuffer));
		Task_sleep(500);

		UART_write(uart, hm10_at_imme1, strlen(hm10_at_imme1));
		UART_read(uart, rxBuffer, sizeof(rxBuffer));
		cli_printf("%s\n", rxBuffer);
		memset(&rxBuffer, 0, sizeof(rxBuffer));
		Task_sleep(1000);

		UART_write(uart, hm10_at_name, strlen(hm10_at_name));
		UART_read(uart, rxBuffer, sizeof(rxBuffer));
		cli_printf("%s\n", rxBuffer);
		memset(&rxBuffer, 0, sizeof(rxBuffer));
		Task_sleep(1000);

		UART_write(uart, hm10_at, strlen(hm10_at));
		UART_read(uart, rxBuffer, sizeof(rxBuffer));
		cli_printf("%s\n", rxBuffer);
		Task_sleep(1000);

		if(!strcmp("OK", rxBuffer)){
			hm10_initialised = 1;
			UART_write(uart, hm10_at_start, strlen(hm10_at_start));

			return 1; //modem can now communicate with us
		}else{
			return 0;
		}
	}else{
		return 0;
	}
}

void hm10_send(char * tx_buffer, int tx_size){
	if(hm10_initialised){
		UART_write(uart, tx_buffer, tx_size);
	}else{
		cli_printf("hm10 not init\n");
	}
}


void hm10_end(){
	UART_close(uart);
	uart = NULL;
}



