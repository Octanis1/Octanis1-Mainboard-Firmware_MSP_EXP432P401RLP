/*
 *  File: sim800.c
 *  Description: GSM modem driver
 *  Author: Sam Sulaimanov
 */

#include "../../../Board.h"

#include "sim800.h"
#include "../rockblock_gsm.h"

#define SIM800_BAUD_RATE 9600
#define SIM800_READ_TIMEOUT 1000
#define SIM800_RXBUFFER_SIZE 100
#define SIM800_TRASHBUFFER_SIZE 32 //required at least 25...


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
static const char sim800_at_smgs[] = "AT+CMGS=\"+41792826160\"\r";
static const char sim800_ctrl_z[] = "\x1A";
static const char sim800_testsms[] = "\xD test \xD";

//sms rx
static const char sim800_at_cmgl[] = "AT+CMGL=\"ALL\"\r\n"; //list all SMS in memory
static const char sim800_at_cmgr1[] = "AT+CMGR=1\r\n"; //read first sms
static const char sim800_at_cmgd1[] = "AT+CMGD=1\r\n"; //delete first sms


static UART_Handle uart;
static UART_Params uartParams;
static int sim800_initialised = 0;
static int sim800_locked = 0;

static char sim800_rxBuffer[SIM800_RXBUFFER_SIZE];
static char sim800_trashBuffer[SIM800_TRASHBUFFER_SIZE];


void sim800_send_sms(char * tx_buffer, int tx_size){

	 if(sim800_initialised  && !sim800_locked){
		sim800_locked = 1;

		UART_write(uart, sim800_at_smgs, strlen(sim800_at_smgs));
		UART_read(uart, sim800_trashBuffer, sizeof(sim800_trashBuffer)); //read '\r\n>'

		UART_write(uart, tx_buffer, tx_size);

		memset(sim800_trashBuffer, 0, sizeof(sim800_trashBuffer));
		UART_write(uart, sim800_ctrl_z, sizeof(sim800_ctrl_z));

		while(strncmp("\r\n+CMGS", sim800_trashBuffer, 7)) //wait for confirmation...
		{
			UART_read(uart, sim800_trashBuffer, sizeof(sim800_trashBuffer));
			static int waitcount = 0;
			waitcount++;
			if(waitcount>20)
			{
				//something went wrong.. (sms may have been received simultaneously.
				Task_sleep(10000);
				UART_read(uart, sim800_trashBuffer, sizeof(sim800_trashBuffer));
				break;
			}
		}

		serial_printf(cli_stdout, "sms:%s", sim800_trashBuffer);

		Task_sleep(100);

		sim800_locked = 0;
	 }else{
		serial_printf(cli_stdout, "sim800 not initialised",0);
	}

}

const char* sim800_get_battery_voltage(){
	memset(sim800_rxBuffer, 0, sizeof(sim800_rxBuffer));

	if(sim800_initialised && !sim800_locked){
		UART_write(uart, sim800_at_cbc, sizeof(sim800_at_cbc));
		UART_read(uart, sim800_rxBuffer, sizeof(sim800_rxBuffer));
		Task_sleep(500);
		return sim800_rxBuffer;
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
	memset(sim800_rxBuffer, 0, sizeof(sim800_rxBuffer));

	if(sim800_open()){
		UART_write(uart, sim800_at_echo_off, sizeof(sim800_at_echo_off));
		UART_read(uart, sim800_rxBuffer, sizeof(sim800_rxBuffer));
		Task_sleep(500);

		memset(sim800_rxBuffer, 0, sizeof(sim800_rxBuffer));
		UART_write(uart, sim800_at_echo_off, sizeof(sim800_at_echo_off));
		UART_read(uart, sim800_rxBuffer, sizeof(sim800_rxBuffer));

		serial_printf(cli_stdout, "%s", sim800_rxBuffer);
		Task_sleep(500);

		if(!strcmp("\r\nOK\r\n", sim800_rxBuffer)){
			sim800_initialised = 1;

			UART_write(uart, sim800_at_smgf, sizeof(sim800_at_smgf)); // put into text mode.
			Task_sleep(500);
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
			memset(sim800_rxBuffer, 0, sizeof(sim800_rxBuffer));
			UART_write(uart, init_http_cmdseq[i], strlen(init_http_cmdseq[i]));
			UART_read(uart, sim800_rxBuffer, sizeof(sim800_rxBuffer));
			Task_sleep(200);
		}


	}
}

void sim800_buffermessage_http(char * tx_buffer, int tx_size){
	if(!sim800_initialised){
		serial_printf(cli_stdout, "sim800 not initialised",0);
	}else{
		memset(sim800_rxBuffer, 0, sizeof(sim800_rxBuffer));

		UART_write(uart, sim800_at_httpdata, sizeof(sim800_at_httpdata));
		Task_sleep(600);
		UART_write(uart, tx_buffer, tx_size);
		UART_read(uart, sim800_rxBuffer, sizeof(sim800_rxBuffer));
		serial_printf(cli_stdout, "%s", sim800_rxBuffer);


		Task_sleep(11000);
		UART_write(uart, sim800_at_httpaction, strlen(sim800_at_httpaction));

		Task_sleep(300);
		UART_write(uart, sim800_at_httpread, strlen(sim800_at_httpread));

		Task_sleep(300);
		UART_write(uart, sim800_at_httpterm, strlen(sim800_at_httpterm));
		UART_read(uart, sim800_rxBuffer, sizeof(sim800_rxBuffer));

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

int sim800_check_rx_sms(char** rx_buffer)
{
	int msg_length = 0;
	//clear rx buffer
	while(UART_read(uart, sim800_trashBuffer, sizeof(sim800_trashBuffer))==sizeof(sim800_trashBuffer));
	memset(sim800_rxBuffer, 0, sizeof(sim800_rxBuffer));

	//list messages in buffer:
	UART_write(uart, sim800_at_cmgl, sizeof(sim800_at_cmgl));
	UART_read(uart, sim800_rxBuffer, sizeof(sim800_rxBuffer));
	if(!strncmp("\r\n+CMGL:", sim800_rxBuffer, 8)) // list contains message!
	{
		char message_index[3] = {0,0,0};
		message_index[0] = sim800_rxBuffer[9];
		if(sim800_rxBuffer[10]!=',')
			message_index[1] = sim800_rxBuffer[10];

		char* start_message_ptr = strchr(&sim800_rxBuffer[7], '\n'); //location where message starts
		*rx_buffer = start_message_ptr;

		if(*rx_buffer)
			*rx_buffer += 1;
		else  //returned null
			return 0;

		char* end_message = strchr(*rx_buffer, '\n'); //location where message ends
		if(end_message == NULL)
			return 0;

		msg_length = end_message - *rx_buffer - 1; //-1 because contains \r

		gsm_execute_command(rx_buffer, msg_length);

		//clear rx buffer

		while(UART_read(uart, sim800_trashBuffer, sizeof(sim800_trashBuffer))==sizeof(sim800_trashBuffer));

		serial_printf(cli_stdout, "Deleting sms:%s", *rx_buffer);


		char sim800_at_cmgdN[12] = "AT+CMGD="; //delete N-th sms
		strcat(sim800_at_cmgdN, message_index);
		strcat(sim800_at_cmgdN,"\r\n");

		//delete message (in all cases):
		UART_write(uart, sim800_at_cmgdN, sizeof(sim800_at_cmgdN));
		UART_read(uart, sim800_rxBuffer, sizeof(sim800_rxBuffer));
	}

	return msg_length;
}
