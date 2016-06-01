/*
 *  File: cli.c
 *  Description: Provides a command line interface and offers possibility to register function as new command
 *
 *  Author:
 */

/* Board Header files */
#include "../../Board.h"

#include "cli.h"
#include "log.h"
#include "../peripherals/gps.h"
#include "../peripherals/navigation.h"
#include "../peripherals/hal/rockblock.h"

#include "../peripherals/comm.h"
//#include "system.h"
//#include "log.h"
//#include "../peripherals/gps.h"
//#include "../peripherals/navigation.h"
//#include "../peripherals/hal/rockblock.h"

#include "../lib/printf.h"
#include "../peripherals/hal/sim800.h"
#include <serial.h>
#include <serial_printf.h>
#include <shell.h>
#include "log_message.h"
//mavlink wire protocol
#include "../lib/mavlink/common/mavlink.h"

//which uart index to use for the CLI
#define CLI_UART Board_UART0_DEBUG
//buffer sizes
//#define CLI_BUFFER 300
//#define PRINTF_BUFFER 300

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
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.writeMode = UART_MODE_BLOCKING;
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

////can be called from any function to queue output strings (currently only integer support)
//void cli_print(char *print_format, ...){
//	static char printf_output_buffer[PRINTF_BUFFER];
//	char *strp = &printf_output_buffer[0];
//
//	// clear buffer from any previous messages
//	memset(strp, 0, sizeof(printf_output_buffer));
//
//	va_list ap;
//	va_start(ap, print_format);
//	tfp_format(&strp, putc_callback, print_format, ap);
//	putc_callback(&strp, 0);
//	va_end(ap);
//
//	//post message to mailbox
//    Mailbox_post(cli_print_mailbox, printf_output_buffer, BIOS_NO_WAIT);  //from this context, timeouts are not allowed
//}
//

//
////allows sending log messages to the console
//void cli_print_task(){
//	char printf_output_buffer[PRINTF_BUFFER];
//
//    //clear buffer from any previous messages
//	memset(&printf_output_buffer[0], 0, sizeof(printf_output_buffer));
//
//	while(1){
//
//		while(Mailbox_pend(cli_print_mailbox, printf_output_buffer, BIOS_WAIT_FOREVER)){
//			//while mailbox contains messages, print out to uart
//			if(uart != NULL){
//				//print message
//				UART_write(uart, printf_output_buffer, sizeof(printf_output_buffer));
//			}
//
//		}
//	}
//}

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

//TODO:replace this part:							////blocks until command received
							//command_length = UART_read(uart, &input, sizeof(input));
							//if(command_length > 1){ //avoid commands that might just be a remaining \n or \r
							//	answer_required = comm_process_command(input, command_length, txdata, &tx_stringlength, DESTINATION_DEBUG_UART);
							//	if(answer_required)
							//	{
							//		serial_printf(stdout, "%s\n",txdata);
							//	}
							//}


//		if(strncmp ("mot", input, 3) == 0){ //motor command was sent
//			if(navigation_bypass(input[3],(input[4]-'0')))
//				tfp_sprintf(output, "okm\n");
//			else
//				tfp_sprintf(output, "inv\n");
//			UART_write(uart, output, sizeof(output));
//		}
//		else if(strcmp("gps\n", input) == 0){
//		   tfp_sprintf(output, "fq %d", gps_get_fix_quality());
//		   UART_write(uart, output, sizeof(output));
//		}else if(strcmp("lat\n", input) == 0){
//			   ftoa(gps_get_lat(), output, 4);
//			   UART_write(uart, output, sizeof(output));
//		}else if(strcmp("lon\n", input) == 0){
//			   ftoa(gps_get_lon(), output, 4);
//			   UART_write(uart, output, sizeof(output));
//		}else if(strcmp("sat\n", input) == 0){
//			   tfp_sprintf(output, "sat %d \n", gps_get_satellites_tracked());
//			   UART_write(uart, output, sizeof(output));
//		}else if(strcmp("valid\n", input) == 0){
//			   tfp_sprintf(output, "valid %d \n", gps_get_validity());
//			   UART_write(uart, output, sizeof(output));
//		}else if(strcmp("hdop\n", input) == 0){
//			   tfp_sprintf(output, "hdop %d \n", gps_get_hdop());
//			   UART_write(uart, output, sizeof(output));
//		}else if(strcmp("lastgps\n", input) == 0){
//			   tfp_sprintf(output, "lu %d \n", gps_get_last_update_time());
//			   UART_write(uart, output, sizeof(output));
//		}else if(strcmp("tasks\n", input) == 0){
//			   system_listTasks();
//		}else if(strcmp("rbs\n", input) == 0){
//			   tfp_sprintf(output, "rb sleep? %d \n", rockblock_get_sleep_status());
//			   UART_write(uart, output, sizeof(output));
//		}else if(strcmp("rbn\n", input) == 0){
//			   tfp_sprintf(output, "rb net? %d \n", rockblock_get_net_availability());
//			   UART_write(uart, output, sizeof(output));
//		}else if (strcmp("logrst\n", input) == 0){
//               log_reset();
//               tfp_sprintf(output, "ok");
//               UART_write(uart, output, sizeof(output));
//        }else if (strcmp("logpos\n", input) == 0){
//               tfp_sprintf(output, "logpos %u", log_write_pos());
//               UART_write(uart, output, sizeof(output));
//        }
//		else if(strcmp("hoff\n", input) == 0){
//			   hx1_off();
//		}else if(strcmp("hon\n", input) == 0){
//			   hx1_on();
//		}


void cmd_hdop(SerialDevice *io, int argc, char *argv[])
{
    serial_printf(io, "hdop %d \n", gps_get_hdop());
}

void cmd_lastgps(SerialDevice *io, int argc, char *argv[])
{
    serial_printf(io, "lu %d \n", gps_get_last_update_time());
}

static void task_print(SerialDevice *dev, Task_Handle task)
{
    Task_Stat stat;
    Task_stat(task, &stat);
    const char *name = Task_Handle_name(task);
    const char *mode = "INVALID";
    switch (stat.mode) {
    case Task_Mode_RUNNING: // Task is currently executing
        mode = "RUNNING";
        break;
    case Task_Mode_READY: // Task is scheduled for execution
        mode = "READY";
        break;
    case Task_Mode_BLOCKED: // Task is suspended from execution
        mode = "BLOCKED";
        break;
    case Task_Mode_TERMINATED: // Task is terminated from execution
        mode = "TERMINATED";
        break;
    case Task_Mode_INACTIVE:  // Task is on inactive task list
        mode = "INACTIVE";
        break;
    };
    serial_printf(dev, "%s: %d, %s, stack: %u/%u\r\n", name, stat.priority,
                  mode, stat.used, stat.stackSize);
}

void cmd_tasks(SerialDevice *io, int argc, char *argv[])
{
    serial_printf(io, "name: priority, mode, stack: used/size\r\n");
    serial_printf(io, "static tasks:\r\n");
    // static tasks
    Task_Object *task;
    int i;
    for (i = 0; i < Task_Object_count(); i++) {
        task = Task_Object_get(NULL, i);
        task_print(io, task);
    }
    // dymanmically allocated tasks
    serial_printf(io, "dynamic tasks:\r\n");
    task = Task_Object_first();
    while (task) {
        task_print(io, task);
        task = Task_Object_next(task);
    }
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


void mavlink_rx(SerialDevice *dev){

	mavlink_message_t msg;
	mavlink_status_t status;
	int c;

	while((c = serial_getc(dev)) >= 0) {
		if(mavlink_parse_char(CHANNEL_APP_UART, (uint8_t)c, &msg, &status)){
			 // --> deal with received message...
		}
	}
}

SerialDevice *stdout;
static UART_SerialDevice cli_uart;
static int cli_uart_initialized = 0;

// public function, must be called by every task using serial_printf before calling it for the first time.
void cli_init()
{
	if(!(cli_uart_initialized))
	{
		cli_uart_init(&cli_uart);
		stdout = (SerialDevice *)&cli_uart;
		cli_uart_initialized = 1;

		log_info("boot");
	}
}

//runs with lowest priority
void cli_task(){

    while (1) {
	#ifndef MAVLINK_ON_UART0_ENABLED
		serial_printf((SerialDevice *)&cli_uart, "octanis Rover Console:\r\n");
		shell(commands, (SerialDevice *)&cli_uart);
	#else
		mavlink_rx((SerialDevice *)&cli_uart);
    #endif
    }


}
