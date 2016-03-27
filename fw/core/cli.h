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

//swi to process print requests from other modules
void cli_print_swi();

#endif
