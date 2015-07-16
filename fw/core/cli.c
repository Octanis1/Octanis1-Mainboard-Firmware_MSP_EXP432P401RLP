/*
 *  File: cli.c
 *  Description: Provides a command line interface and offers possibility to register function as new command
 *  Author:
 */
#include "cli.h"
#include <ti/drivers/UART.h>
#include <ti/drivers/uart/UARTMSP432.h>

//which uart index to use for the CLI
#define CLI_UART 0
//buffer to fill before data goes out
#define CLI_BUFFER 512


static UART_Handle handle;
static UART_Params params;

void cli_init(){

	//open uart
	UART_Params_init(&params);
	params.baudRate = 9600;
	params.writeDataMode = UART_DATA_TEXT;
	params.readDataMode = UART_DATA_TEXT;
	params.readReturnMode = UART_RETURN_NEWLINE;
	params.readEcho = UART_ECHO_ON;
	handle = UART_open(CLI_UART, &params);
	
	if (!handle) {
		//System_printf("UART did not open");
	}


}

//allows sending log messages to the console
void cli_print(){

}


//runs with lowest priority
void cli_task(){
	//listen to strings from uart

	const unsigned char hello[] = "Hello World\n";
	UART_write(handle, hello, sizeof(hello));


}
