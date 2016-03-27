/*
 *  File: sim800.c
 *  Description: GSM modem driver
 *  Author: Sam Sulaimanov
 */

#include "../../../Board.h"

#include "sim800.h"

#define SIM800_BAUD_RATE 9600
#define SIM800_READ_TIMEOUT 3000
#define SIM800_RXBUFFER_SIZE 100

static const char sim800_at[] = "AT\r\n";
static const char sim800_at_restart[] = "AT+CFUN=1,1";
static const char sim800_at_echo_off[] = "ATE0\r\n";
static const char sim800_at_sapbr_apn[] = "AT+SAPBR=3,1,\"APN\",\"gprs.swisscom.ch\"\r\n";
static const char sim800_at_sapbr[] = "AT+SAPBR=1,1\r\n"; //gives an error if already executed
static const char sim800_at_httpinit[] = "AT+HTTPINIT\r\n";
static const char sim800_at_httppara_url[] = "at+httppara=\"url\",\"http://basestation.octanis.org:9999/gsm_packets/Aeho4Zuze7Muc1\"\r\n";
static const char sim800_at_httppara_cid[] = "at+httppara=\"cid\",1\r\n";
static const char sim800_at_httppara_content[] = "AT+HTTPPARA=\"CONTENT\",\"text/plain\"\r\n";
static const char sim800_at_httpdata[] = "at+httpdata=100,2000\r\n";
static const char sim800_at_httpaction[] = "AT+HTTPACTION=1\r\n";
static const char sim800_at_httpread[] = "AT+HTTPREAD\r\n";
static const char sim800_at_httpterm[] = "AT+HTTPTERM\r\n";


static UART_Handle uart;
static UART_Params uartParams;
static int sim800_initialised = 0;

int sim800_open(){

	UART_Params_init(&uartParams);
	uartParams.writeDataMode = UART_DATA_BINARY;
	uartParams.readDataMode = UART_DATA_BINARY;
	uartParams.readReturnMode = UART_RETURN_FULL;
	uartParams.readTimeout = SIM800_READ_TIMEOUT;
	uartParams.readEcho = UART_ECHO_OFF;
	uartParams.baudRate = SIM800_BAUD_RATE;

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
int sim800_begin(){
	char rxBuffer[SIM800_RXBUFFER_SIZE];
	memset(&rxBuffer, 0, sizeof(rxBuffer));

	if(sim800_open()){
		UART_write(uart, sim800_at_echo_off, sizeof(sim800_at_echo_off));
		UART_read(uart, rxBuffer, sizeof(rxBuffer));
		Task_sleep(500);

		memset(&rxBuffer, 0, sizeof(rxBuffer));
		UART_write(uart, sim800_at_echo_off, sizeof(sim800_at_echo_off));
		UART_read(uart, rxBuffer, sizeof(rxBuffer));

		cli_printf("%s", rxBuffer);
		Task_sleep(500);

		if(!strcmp("\r\nOK\r\n", rxBuffer)){
			sim800_initialised = 1;
			return 1; //modem can now communicate with us
		}else{
			return 0;
		}
	}else{
		return 0;
	}
}

void sim800_end(){
	UART_close(uart);
	uart = NULL;
}


void sim800_init_http(){
	if(!sim800_initialised){
		cli_printf("sim800 not initialised",0);
	}else{
		char rxBuffer[SIM800_RXBUFFER_SIZE];
		int i;

		const char *init_http_cmdseq[] = {
			sim800_at_sapbr_apn,
			sim800_at_sapbr,
			sim800_at_httpinit,
			sim800_at_httppara_url,
			sim800_at_httppara_cid,
			sim800_at_httppara_content
		};

		for(i=0; i<6; i++){
			memset(&rxBuffer, 0, sizeof(rxBuffer));
			UART_write(uart, init_http_cmdseq[i], strlen(init_http_cmdseq[i])); //hardcoded size...
			UART_read(uart, rxBuffer, sizeof(rxBuffer));

			//cli_printf("%d - %s", i, rxBuffer);
			Task_sleep(200);
		}


	}
}

void sim800_buffermessage_http(char * tx_buffer, int tx_size, int download_time){
	if(!sim800_initialised){
		cli_printf("sim800 not initialised",0);
	}else{
		char rxBuffer[SIM800_RXBUFFER_SIZE];
		memset(&rxBuffer, 0, sizeof(rxBuffer));

		UART_write(uart, sim800_at_httpdata, sizeof(sim800_at_httpdata));
		Task_sleep(600);
		UART_write(uart, "na na na na na", strlen("na na na na na"));
		UART_read(uart, rxBuffer, sizeof(rxBuffer));
		cli_printf("%s", rxBuffer);
		memset(&rxBuffer, 0, sizeof(rxBuffer));

		Task_sleep(5000);

		UART_write(uart, sim800_at_httpaction, strlen(sim800_at_httpaction));
		UART_read(uart, rxBuffer, sizeof(rxBuffer));
		cli_printf("%s", rxBuffer);
		memset(&rxBuffer, 0, sizeof(rxBuffer));

		Task_sleep(300);
		UART_write(uart, sim800_at_httpread, strlen(sim800_at_httpread));
		UART_read(uart, rxBuffer, sizeof(rxBuffer));
		cli_printf("%s", rxBuffer);
		memset(&rxBuffer, 0, sizeof(rxBuffer));

		Task_sleep(300);
		UART_write(uart, sim800_at_httpterm, strlen(sim800_at_httpterm));
		UART_read(uart, rxBuffer, sizeof(rxBuffer));
		cli_printf("%s", rxBuffer);
		memset(&rxBuffer, 0, sizeof(rxBuffer));

		Task_sleep(300);

	}
}

void sim800_send_http(char * tx_buffer, int tx_size, int download_time){
	if(!sim800_initialised){
		cli_printf("sim800 not initialised",0);
	}else{
		sim800_init_http();
		sim800_buffermessage_http("test",1,1);
	}
}
