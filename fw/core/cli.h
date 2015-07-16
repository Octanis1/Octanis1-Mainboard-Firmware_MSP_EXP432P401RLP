/*
 *  File: cli.h
 *  Description: Provides a command line interface and offers possibility to register function as new command
 *  Author:
 */


//setup uart, connect with stdin and stdout
void cli_init();

//register a command line command here. will be called when "commandstring [argument]" was typed into cli.
void cli_register_command(void (*command_cb)(char*), char* commandstring);

//runs with lowest priority
void cli_task();

//allows sending log messages to the console
void cli_print();
