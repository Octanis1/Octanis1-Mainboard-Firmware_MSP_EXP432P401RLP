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
static const char sim800_at_restart[] = "AT+CFUN=1,1\r\n";
static const char sim800_at_cbc[] = "AT+CBC\r\n";
static const char sim800_at_echo_off[] = "ATE0\r\n";
static const char sim800_at_sapbr_apn[] = "AT+SAPBR=3,1,\"APN\",\"gprs.swisscom.ch\"\r\n";
static const char sim800_at_sapbr[] = "AT+SAPBR=1,1\r\n"; //gives an error if already executed
static const char sim800_at_httpinit[] = "AT+HTTPINIT\r\n";


#ifndef CAMERA_BOARD
static const char sim800_at_httppara_url[] = "at+httppara=\"url\",\"http://basestation.octanis.org:9999/gsm_packets/Aeho4Zuze7Muc1?board=2\"\r\n";
#else
static const char sim800_at_httppara_url[] = "at+httppara=\"url\",\"http://basestation.octanis.org:9999/gsm_packets/Aeho4Zuze7Muc1?board=1\"\r\n";
#endif


static const char sim800_at_httppara_cid[] = "at+httppara=\"cid\",1\r\n";
static const char sim800_at_httppara_content_stream[] = "AT+HTTPPARA=\"CONTENT\",\"application/octet-stream\"\r\n";
static const char sim800_at_httppara_content_text[] = "AT+HTTPPARA=\"CONTENT\",\"text/plain\"\r\n";

//http
static const char sim800_at_httpdata[] = "at+httpdata=10000,10000\r\n";
static const char sim800_at_httpaction[] = "AT+HTTPACTION=1\r\n";
static const char sim800_at_httpread[] = "AT+HTTPREAD\r\n";
static const char sim800_at_httpterm[] = "AT+HTTPTERM\r\n";

//sms
static const char sim800_at_smgf[] = "AT+CMGF=1\r\n";
static const char sim800_at_smgs[] = "AT+CMGS=\"+41798690718\"\r";
static const char sim800_ctrl_z[] = "\x1A";
static const char sim800_testsms[] = "\xD test \xD";


static UART_Handle uart;
static UART_Params uartParams;
static int sim800_initialised = 0;
static int sim800_locked = 0;



void sim800_send_sms(char * tx_buffer, int tx_size){
	char rxBuffer[SIM800_RXBUFFER_SIZE];

	 if(sim800_initialised  && !sim800_locked){
		sim800_locked = 1;

		UART_write(uart, sim800_at_smgf, sizeof(sim800_at_smgf));
		Task_sleep(500);

		UART_write(uart, sim800_at_smgs, strlen(sim800_at_smgs));
		Task_sleep(1000);

		UART_write(uart, tx_buffer, tx_size);
		Task_sleep(5000);

		memset(&rxBuffer, 0, sizeof(rxBuffer));
		UART_write(uart, sim800_ctrl_z, sizeof(sim800_ctrl_z));
		UART_read(uart, rxBuffer, sizeof(rxBuffer));
		serial_printf(cli_stdout, "sms:%s", rxBuffer);

		sim800_locked = 0;
	 }else{
		serial_printf(cli_stdout, "sim800 not initialised",0);
	}

}

const char* sim800_get_battery_voltage(){
	static char rxBuffer[SIM800_RXBUFFER_SIZE];
	memset(&rxBuffer, 0, sizeof(rxBuffer));

	if(sim800_initialised && !sim800_locked){
		UART_write(uart, sim800_at_cbc, sizeof(sim800_at_cbc));
		UART_read(uart, rxBuffer, sizeof(rxBuffer));
		Task_sleep(500);
		return rxBuffer;
	}else{
		serial_printf(cli_stdout, "sim800 not initialised",0);
		return 0;
	}
}




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

		serial_printf(cli_stdout, "%s", rxBuffer);
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


void sim800_init_http(SIM800_MIME mime_type){
	if(!sim800_initialised){
		serial_printf(cli_stdout, "sim800 not initialised",0);
	}else{
		char rxBuffer[SIM800_RXBUFFER_SIZE];
		int i;

		const char *init_http_cmdseq[] = {
			sim800_at_sapbr_apn,
			sim800_at_sapbr,
			sim800_at_httpinit,
			sim800_at_httppara_url,
			sim800_at_httppara_cid,
			""
		};

		//set selected mime type
		if(mime_type == MIME_OCTET_STREAM){
			init_http_cmdseq[5] = sim800_at_httppara_content_stream;
		}else{
			init_http_cmdseq[5] = sim800_at_httppara_content_text;
		}


		for(i=0; i<6; i++){
			memset(&rxBuffer, 0, sizeof(rxBuffer));
			UART_write(uart, init_http_cmdseq[i], strlen(init_http_cmdseq[i]));
			UART_read(uart, rxBuffer, sizeof(rxBuffer));
			Task_sleep(200);
		}


	}
}

void sim800_buffermessage_http(char * tx_buffer, int tx_size){
	if(!sim800_initialised){
		serial_printf(cli_stdout, "sim800 not initialised",0);
	}else{
		char rxBuffer[SIM800_RXBUFFER_SIZE];
		memset(&rxBuffer, 0, sizeof(rxBuffer));

		UART_write(uart, sim800_at_httpdata, sizeof(sim800_at_httpdata));
		Task_sleep(600);
		UART_write(uart, tx_buffer, tx_size);
		UART_read(uart, rxBuffer, sizeof(rxBuffer));
		serial_printf(cli_stdout, "%s", rxBuffer);


		Task_sleep(11000);
		UART_write(uart, sim800_at_httpaction, strlen(sim800_at_httpaction));

		Task_sleep(300);
		UART_write(uart, sim800_at_httpread, strlen(sim800_at_httpread));

		Task_sleep(300);
		UART_write(uart, sim800_at_httpterm, strlen(sim800_at_httpterm));
		UART_read(uart, rxBuffer, sizeof(rxBuffer));

	}
}

void sim800_send_http(char * tx_buffer, int tx_size, SIM800_MIME mime_type){
	if(!sim800_initialised){
		serial_printf(cli_stdout, "sim800 not initialised",0);
	}else{
		sim800_locked = 1;
		sim800_init_http(mime_type);
		sim800_buffermessage_http(tx_buffer, tx_size);
		sim800_locked = 0;
	}
}
