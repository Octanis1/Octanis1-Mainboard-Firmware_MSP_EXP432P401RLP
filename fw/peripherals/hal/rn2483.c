/*
 *  File: rn2483.c
 *  Description: Model for LoRaWAN tranceiver "Microchip RN2483"
 *  Author: Sam
 */

#include "../../../Board.h"
#include "rn2483.h"
#include "../comm.h"

#define RN2483_RXBUFFER_SIZE 20
#define RN2483_READ_TIMEOUT 1000
#define RN2483_BAUD_RATE 57600

static UART_Handle uart;
static UART_Params uartParams;
static int rn2483_initialised = 1;

#ifdef CONFIG_MODE
//responses from the modem are "encapsulated" in "\r\n"
#define RN2483_CONFIG_LENGTH 50 //maximum length of the below 4 strings
static const char rn_set_nwkskey[] ="mac set nwkskey A4F613D0D65CC3DA3D874D82E12265B8\r\n";
static const char rn_set_appskey[] ="mac set appskey 32BC3ABB9E1ADC0594811AEB1CCA704F\r\n";
static const char rn_set_devaddr[] ="mac set devaddr 08050046\r\n";
static const char rn_set_deveui[] ="mac set deveui F03D291000000046\r\n";
static const char rn_save[] ="mac save\r\n";
#endif


static const char rn_get_devaddr[] = "mac get devaddr\r\n";
static const char rn_get_deveui[] = "mac get deveui\r\n";
static const char rn_get_nwkskey[] = "mac get nwkskey\r\n";
static const char rn_get_appskey[] = "mac get appskey\r\n";


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

	uart = UART_open(Board_UART3_LORACOMM, &uartParams);

	if (uart == NULL) {
		return 0;
	}else{
		return 1;
	}
}


//must be called from within a task - this function will block!
int rn2483_begin(){

	//	reset the rn2483
    GPIO_write(Board_LORA_RESET_N, 0);

#ifndef CONFIG_MODE
	if(!rn2483_open()){
		cli_printf("rn2483 open error \n",0);
		return 0;
	}else{
		cli_printf("rn2483 opened\n",0);
	}
#endif

	GPIO_write(Board_LORA_RESET_N, 1);
	char rxBuffer[RN2483_RXBUFFER_SIZE];

//	UART_write(uart, rn_reset, sizeof(rn_reset));
//	UART_read(uart, rxBuffer, sizeof(rxBuffer));

	Task_sleep(500);
	UART_read(uart, rxBuffer, sizeof(rxBuffer)); //clean the reset message
	UART_read(uart, rxBuffer, sizeof(rxBuffer));
	memset(&rxBuffer, 0, sizeof(rxBuffer));


//	/*check parameters*/
//	Task_sleep(500);
//	UART_write(uart, rn_get_devaddr, sizeof(rn_get_devaddr));
//	UART_read(uart, rxBuffer, sizeof(rxBuffer));
//	Task_sleep(500);
//
//	UART_write(uart, rn_get_deveui, sizeof(rn_get_deveui));
//	UART_read(uart, rxBuffer, sizeof(rxBuffer));
//	Task_sleep(500);



	UART_write(uart, rn_join, sizeof(rn_join));
	UART_read(uart, rxBuffer, sizeof(rxBuffer));

	Task_sleep(500);

	if(!strcmp("ok\r\naccepted\r\n", rxBuffer)){
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

#ifdef CONFIG_MODE
/*Used to configure the module with network data etc. */
int rn2483_config()
{
	int comm_result = 0;

	comm_result+=(!rn2483_open());

	char rxBuffer[RN2483_RXBUFFER_SIZE];

	Task_sleep(500);
	UART_write(uart, rn_set_devaddr, sizeof(rn_set_devaddr));
	UART_read(uart, rxBuffer, sizeof(rxBuffer));
	Task_sleep(500);
	comm_result += !(rxBuffer[0] == 'o' && rxBuffer[1] == 'k');
	rxBuffer[0] = 0;

	UART_write(uart, rn_set_deveui, sizeof(rn_set_deveui));
	UART_read(uart, rxBuffer, sizeof(rxBuffer));
	Task_sleep(500);
	comm_result += !(rxBuffer[0] == 'o' && rxBuffer[1] == 'k');
	rxBuffer[0] = 0;

	UART_write(uart, rn_set_nwkskey, sizeof(rn_set_nwkskey));
	UART_read(uart, rxBuffer, sizeof(rxBuffer));
	Task_sleep(500);
	comm_result += !(rxBuffer[0] == 'o' && rxBuffer[1] == 'k');
	rxBuffer[0] = 0;

	UART_write(uart, rn_set_appskey, sizeof(rn_set_appskey));
	UART_read(uart, rxBuffer, sizeof(rxBuffer));
	Task_sleep(500);
	comm_result += !(rxBuffer[0] == 'o' && rxBuffer[1] == 'k');
	rxBuffer[0] = 0;

	UART_write(uart, rn_save, sizeof(rn_save));
	Task_sleep(500);
	UART_read(uart, rxBuffer, sizeof(rxBuffer));
	Task_sleep(500);
	comm_result += !(rxBuffer[0] == 'o' && rxBuffer[1] == 'k');

	return comm_result;
}
#endif



int rn2483_send_receive(char * tx_buffer, int tx_size)
{
	char rxBuffer[RN2483_RXBUFFER_SIZE];
	memset(&rxBuffer, 0, sizeof(rxBuffer));
	char txBuffer[tx_size+18]; //18 for the aditionnal lora commands

	if(!rn2483_initialised) return 0;

	strcat(txBuffer, rn_txstart);
	strcat(txBuffer, tx_buffer);
	strcat(txBuffer, rn_txstop);

	int tx_ret = UART_write(uart, txBuffer, strlen(txBuffer));
	int rx_ret = UART_read(uart, rxBuffer, sizeof(rxBuffer)); //TODO: after 1st "ok", it receives a \n before "ok"

	cli_printf("tx %d \n", tx_ret);

	if(!strcmp("ok\r\n", rxBuffer))
	{
		return 1; //modem can now communicate with us
	}
	else
	{
		return 0;
	}


}


