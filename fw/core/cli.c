/*
 *  File: cli.c
 *  Description: Provides a command line interface and offers possibility to register function as new command
 *  Author:
 */

/* Board Header files */
#include "../../Board.h"

#include "cli.h"
#include "../lib/printf.h"


//which uart index to use for the CLI
#define CLI_UART Board_UART0_DEBUG
//buffer sizes
#define CLI_BUFFER 10
#define PRINTF_BUFFER 20


static Semaphore_Handle cli_print_sem;

static UART_Handle uart = NULL;
static UART_Params uartParams;

static char printf_output_buffer[PRINTF_BUFFER];


void cli_init(){

    /* Create a UART with data processing off. */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_TEXT;
    uartParams.readReturnMode = UART_RETURN_NEWLINE;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.baudRate = 9600;
    uart = UART_open(CLI_UART, &uartParams);

    if (uart == NULL) {
        System_abort("Error opening the UART");
    }

    //create semaphore for print requests
    cli_print_sem = Semaphore_create(0, NULL, NULL);
}



//can be called from any function to queue output strings (currently only integer support)
void cli_printf(char *print_format, int number){
    //clear buffer from any previous messages
	memset(&printf_output_buffer[0], 0, sizeof(printf_output_buffer));

    tfp_sprintf(printf_output_buffer, print_format, number);

	Semaphore_post(cli_print_sem);
}



//allows sending log messages to the console
void cli_print_task(){

	while(1){

		if(uart != NULL){
			Semaphore_pend(cli_print_sem, BIOS_WAIT_FOREVER);

			//print message
		    UART_write(uart, printf_output_buffer, sizeof(printf_output_buffer));
		}

	}


}


//runs with lowest priority
void cli_task(){

	   	char input[CLI_BUFFER];
	    char output[CLI_BUFFER];
	    const char consolePrompt[] = "octanis Rover Console:\r\n";

	    //initialises UART0 handle
	    cli_init();

	    //prints welcome message
	    UART_write(uart, consolePrompt, sizeof(consolePrompt));

	    /* loop forever */
	    while (1) {

	        //clears buffers
	        memset(&input[0], 0, sizeof(input));
	        memset(&output[0], 0, sizeof(output));

	        //blocks until command received
	        UART_read(uart, &input, sizeof(input));


	        if(strcmp("load\n", input) == 0){
	           tfp_sprintf(output, "Load %d", 9);
	           UART_write(uart, output, sizeof(output));

	        }else{

	        }

	        UART_write(uart, input, sizeof(input));


	    }


}
