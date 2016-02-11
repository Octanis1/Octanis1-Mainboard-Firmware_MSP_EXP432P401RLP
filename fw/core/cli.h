/*
 *  File: cli.h
 *  Description: Provides a command line interface and offers possibility to register function as new command
 *  Author:
 */


#ifndef __CLI_H
#define __CLI_H

//register a command line command here. will be called when "commandstring [argument]" was typed into cli.
void cli_register_command(void (*command_cb)(char*), char* commandstring);

//runs with lowest priority
void cli_task();

//allows sending log messages to the console
void cli_printf(char *print_format, ...);

//swi to process print requests from other modules
void cli_print_swi();

#endif
