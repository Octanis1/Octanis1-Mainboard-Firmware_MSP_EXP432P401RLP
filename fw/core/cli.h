/*
 *  File: cli.h
 *  Description: Provides a command line interface and offers possibility to register function as new command
 *  Author:
 */


#ifndef __CLI_H
#define __CLI_H

//runs with lowest priority
void cli_task();

//allows sending log messages to the console
void cli_printf(char *print_format, ...);

//direct sending of mavlink message of UART0 -> just for testing. we need to replace cli.c with the new shell
void cli_send_mavlink(unsigned char *mavlink_message, int mavlink_message_size);


//swi to process print requests from other modules
void cli_print_swi();

#endif
