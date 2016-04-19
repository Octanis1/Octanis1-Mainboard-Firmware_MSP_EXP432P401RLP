/*
 *  File: cli.c
 *  Description: Provides a command line interface and offers possibility to register function as new command
 *  Author:
 */

/* Board Header files */
#include "../../Board.h"

#include "cli.h"
#include "system.h"
#include "log.h"
#include "../peripherals/gps.h"
#include "../peripherals/navigation.h"
#include "../peripherals/hal/rockblock.h"
#include "../lib/printf.h"
#include "../peripherals/hal/sim800.h"
#include <serial.h>
#include <serial_printf.h>
#include <shell.h>

//which uart index to use for the CLI
#define CLI_UART Board_UART0_DEBUG
//buffer sizes
#define CLI_BUFFER 300
#define PRINTF_BUFFER 300

typedef struct {
    const struct serial_function_table *fntab;
    UART_Handle uart;
} UART_SerialDevice;

size_t uart_serial_write(void *dev, const uint8_t *data, size_t n)
{
	return UART_write(((UART_SerialDevice *)dev)->uart, data, n);
}

size_t uart_serial_read(void *dev, uint8_t *data, size_t n)
{
	return UART_read(((UART_SerialDevice *)dev)->uart, data, n);
}

int uart_serial_putc(void *dev, uint8_t c)
{
	UART_write(((UART_SerialDevice *)dev)->uart, &c, 1);
	return 0;
}

int uart_serial_getc(void *dev)
{
	uint8_t c;
	if (UART_read(((UART_SerialDevice *)dev)->uart, &c, 1) == 1) {
		return c;
	}
	return SERIAL_EOF;
}

const struct serial_function_table UART_SerialDevice_fntab = {
	uart_serial_write,
	uart_serial_read,
	uart_serial_putc,
	uart_serial_getc
};

static UART_Handle uart = NULL;

static void cli_uart_init(UART_SerialDevice *dev) {
	static UART_Params uartParams;

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

    dev->fntab = &UART_SerialDevice_fntab;
    dev->uart = uart;
}


static void putc_callback(void* p,char c){
	*(*((char**)p))++ = c;
}

//can be called from any function to queue output strings (currently only integer support)
void cli_printf(char *print_format, ...){
	static char printf_output_buffer[PRINTF_BUFFER];
	char *strp = &printf_output_buffer[0];

	// clear buffer from any previous messages
	memset(strp, 0, sizeof(printf_output_buffer));

	va_list ap;
	va_start(ap, print_format);
	tfp_format(&strp, putc_callback, print_format, ap);
	putc_callback(&strp, 0);
	va_end(ap);

	//post message to mailbox
    Mailbox_post(cli_print_mailbox, printf_output_buffer, BIOS_NO_WAIT);  //from this context, timeouts are not allowed
}



//allows sending log messages to the console
void cli_print_task(){
	char printf_output_buffer[PRINTF_BUFFER];

    //clear buffer from any previous messages
	memset(&printf_output_buffer[0], 0, sizeof(printf_output_buffer));

	while(1){

		while(Mailbox_pend(cli_print_mailbox, printf_output_buffer, BIOS_WAIT_FOREVER)){
			//while mailbox contains messages, print out to uart
			if(uart != NULL){
				//print message
				UART_write(uart, printf_output_buffer, sizeof(printf_output_buffer));
			}

		}
	}
}

const struct shell_commands commands[];

void cmd_help(SerialDevice *dev, int argc, char *argv[])
{
    const struct shell_commands *c = commands;
    serial_printf(dev, "Commands:");
    while (c->name != NULL && c->function != NULL) {
        serial_printf(dev, " %s", c->name);
        c++;
    }
    serial_printf(dev, "\r\n");
}

void cmd_motor(SerialDevice *io, int argc, char *argv[])
{
    if (argc != 3) {
        serial_printf(io, "wrong number of arguments");
        return;
    }
    if(navigation_bypass(*argv[1], *argv[2]-'0')) {
        serial_printf(io, "okm\n");
    } else {
        serial_printf(io, "inv\n");
    }
}

void cmd_gps(SerialDevice *io, int argc, char *argv[])
{
    serial_printf(io, "fq %d", gps_get_fix_quality());
}

void cmd_lat(SerialDevice *io, int argc, char *argv[])
{
    char buf[15];
    int len = ftoa(gps_get_lat(), buf, 4);
    serial_write(io, (uint8_t *)buf, len);
}

void cmd_lon(SerialDevice *io, int argc, char *argv[])
{
    char buf[15];
    int len = ftoa(gps_get_lon(), buf, 4);
    serial_write(io, (uint8_t *)buf, len);
}

void cmd_sat(SerialDevice *io, int argc, char *argv[])
{
    serial_printf(io, "sat %d \n", gps_get_satellites_tracked());
}

void cmd_valid(SerialDevice *io, int argc, char *argv[])
{
    serial_printf(io, "valid %d \n", gps_get_validity());
}

void cmd_hdop(SerialDevice *io, int argc, char *argv[])
{
    serial_printf(io, "hdop %d \n", gps_get_hdop());
}

void cmd_lastgps(SerialDevice *io, int argc, char *argv[])
{
    serial_printf(io, "lu %d \n", gps_get_last_update_time());
}

void cmd_tasks(SerialDevice *io, int argc, char *argv[])
{
    system_listTasks();
}

void cmd_rbs(SerialDevice *io, int argc, char *argv[])
{
    serial_printf(io, "rb sleep? %d \n", rockblock_get_sleep_status());
}

void cmd_rbn(SerialDevice *io, int argc, char *argv[])
{
    serial_printf(io, "rb net? %d \n", rockblock_get_net_availability());
}

const struct shell_commands commands[] = {
    {"help", cmd_help},
    {"gps", cmd_gps},
    {"lat", cmd_lat},
    {"lon", cmd_lon},
    {"sat", cmd_sat},
    {"valid", cmd_valid},
    {"hdop", cmd_hdop},
    {"lastgps", cmd_lastgps},
    {"tasks", cmd_tasks},
    {"rbs", cmd_rbs},
    {"rbn", cmd_rbn},
    {NULL, NULL}
};

//runs with lowest priority
void cli_task(){
	static UART_SerialDevice cli_uart;
    cli_uart_init(&cli_uart);

    while (1) {
        serial_printf((SerialDevice *)&cli_uart, "octanis Rover Console:\r\n");
        shell(commands, (SerialDevice *)&cli_uart);
    }
}
