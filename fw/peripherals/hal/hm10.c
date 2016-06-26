/*
 *  File: hm10.c
 *  Description: BLE modem driver
 *  Author: Sam Sulaimanov
 */

#include "../../../Board.h"

#include "hm10.h"

#define HM10_BAUD_RATE 9600
#define HM10_READ_TIMEOUT 3000

static const char hm10_at[] = "AT"; //does not require return characters
static const char hm10_at_name[] = "AT+NAMEmars3";
static const char hm10_at_clear[] = "AT+CLEAR"; //clear last connected device
static const char hm10_at_wakestring[] = "I am iron man,I am iron man,I am iron man,I am iron man, I am iron man, I am iron man I am iron I am iron man, I am iron man.";
static const char hm10_at_imme1[] = "AT+IMME1"; //When module is powered on, only respond the AT Command, don’t do anything.
static const char hm10_at_start[] = "AT+START"; //start connecting to devices
static const char hm10_at_pwrm1[] = "AT+PWRM1"; //doesnt go to sleep in this mode
static const char hm10_at_pwrm0[] = "AT+PWRM0"; //sleep mode

static UART_Handle uart;
static UART_Params uartParams;
static int hm10_initialised = 0;

static char 	hm10_rxBuffer[HM10_RXBUFFER_SIZE];
static int	hm10_rxCount;

void hm10_read_callback(UART_Handle handle, void *buf, size_t count);
int hm10_callback;

//must be called from within a task - this function will block!
//returns 1 if modem responds with OK
int hm10_begin(){

	memset(&hm10_rxBuffer, 0, sizeof(hm10_rxBuffer));

	UART_Params_init(&uartParams);
	uartParams.writeDataMode = UART_DATA_BINARY;
	uartParams.readMode = UART_MODE_BLOCKING;
	uartParams.readTimeout = HM10_READ_TIMEOUT;
	uartParams.readDataMode = UART_DATA_BINARY;
	uartParams.readReturnMode = UART_RETURN_FULL;
	uartParams.readEcho = UART_ECHO_OFF;
	uartParams.baudRate = HM10_BAUD_RATE;

	uart = UART_open(Board_UART2_COMM, &uartParams);

	if (uart == NULL) {
		//debug
		GPIO_toggle(Board_LED_GREEN);
		Task_sleep(300);
		GPIO_toggle(Board_LED_GREEN);
		return 0;
	}
	else
	{
		Task_sleep(1000);

		UART_write(uart, hm10_at_wakestring, strlen(hm10_at_wakestring));
		UART_read(uart, hm10_rxBuffer, sizeof(hm10_rxBuffer));
		serial_printf(cli_stdout, "%s\n", hm10_rxBuffer);
		memset(&hm10_rxBuffer, 0, sizeof(hm10_rxBuffer));

		UART_write(uart, hm10_at_pwrm1, strlen(hm10_at_pwrm1));
		UART_read(uart, hm10_rxBuffer, sizeof(hm10_rxBuffer));
		serial_printf(cli_stdout, "%s\n", hm10_rxBuffer);
		memset(&hm10_rxBuffer, 0, sizeof(hm10_rxBuffer));
		Task_sleep(1000);

		UART_write(uart, hm10_at_clear, strlen(hm10_at_clear));
		UART_read(uart, hm10_rxBuffer, sizeof(hm10_rxBuffer));
		serial_printf(cli_stdout, "%s\n", hm10_rxBuffer);
		memset(&hm10_rxBuffer, 0, sizeof(hm10_rxBuffer));
		Task_sleep(500);

		UART_write(uart, hm10_at_imme1, strlen(hm10_at_imme1));
		UART_read(uart, hm10_rxBuffer, sizeof(hm10_rxBuffer));
		serial_printf(cli_stdout, "%s\n", hm10_rxBuffer);
		memset(&hm10_rxBuffer, 0, sizeof(hm10_rxBuffer));
		Task_sleep(1000);

		UART_write(uart, hm10_at_name, strlen(hm10_at_name));
		UART_read(uart, hm10_rxBuffer, sizeof(hm10_rxBuffer));
		serial_printf(cli_stdout, "%s\n", hm10_rxBuffer);
		memset(&hm10_rxBuffer, 0, sizeof(hm10_rxBuffer));
		Task_sleep(1000);

		UART_write(uart, hm10_at, strlen(hm10_at));
		UART_read(uart, hm10_rxBuffer, sizeof(hm10_rxBuffer));
		serial_printf(cli_stdout, "%s\n", hm10_rxBuffer);
		if(!strcmp("OK", hm10_rxBuffer)){
			UART_write(uart, hm10_at_start, strlen(hm10_at_start));
			hm10_end(); // Close UART port to configure Text mode to callback @ newline + CR

			uartParams.writeDataMode = UART_DATA_BINARY;
			/* Experimental: use callback mode such that the comm task is not blocked
			 * while waiting for incoming commands.
			 *
			 * Note: think about using void UART_readCancel 	( 	UART_Handle  	handle	)
			 * WARNING: It is STRONGLY discouraged to call UART_read from its own callback function (UART_MODE_CALLBACK).
			 */
			uartParams.readMode = UART_MODE_CALLBACK;
			uartParams.readCallback = hm10_read_callback;
			uartParams.readTimeout = UART_WAIT_FOREVER; //HM10_READ_TIMEOUT;
			uartParams.readDataMode = UART_DATA_TEXT;
			uartParams.readReturnMode = UART_RETURN_NEWLINE;

			uart = UART_open(Board_UART2_COMM, &uartParams);

			if (uart == NULL)
			{
				//debug
				GPIO_toggle(Board_LED_GREEN);
				Task_sleep(300);
				GPIO_toggle(Board_LED_GREEN);
				return 0;
			}
			else
			{
				hm10_initialised = 1;
				// directly be receptive for commands
				hm10_callback = 0;
				UART_read(uart, hm10_rxBuffer, sizeof(hm10_rxBuffer));
				return 1; //modem can now communicate with us
			}
		}
		else
		{
			return 0;
		}
	}
}

/* function checks if new data has been received via BLE module and starts a new UART_read.
 * Returns 1 if new data was received, 0 if no new data is present.
 */
int hm10_receive(char* rxdata, int* stringlength)
{
	if(hm10_callback == 1)
	{
		(*stringlength) = hm10_rxCount;

		memcpy(rxdata,hm10_rxBuffer,hm10_rxCount);

		hm10_callback = 0;
		if(hm10_initialised)
		{
			//generate next read command
			UART_read(uart, hm10_rxBuffer, sizeof(hm10_rxBuffer));
			return 1;
		}
		else
		{
			serial_printf(cli_stdout, "RX failed. hm10 not init\n");
		}
	}
	return 0;
}


void hm10_send(char * tx_buffer, int tx_size)
{
	if(hm10_initialised){
		UART_write(uart, tx_buffer, tx_size);
		UART_write(uart, "\n", 1);
	}else{
		serial_printf(cli_stdout, "TX failed. hm10 not init\n");
	}
}


void hm10_end(){
	if(uart != NULL)
	{
		hm10_initialised = 0;
		UART_close(uart);
		uart = NULL;
	}
}

void hm10_read_callback(UART_Handle handle, void *buf, size_t count)
{
	hm10_rxCount  = (int)count;
	hm10_callback = 1;
}


