#include <string.h>
#include "shell.h"
#include "serial_printf.h"

#ifndef SHELL_PROMPT
#define SHELL_PROMPT "> "
#endif

#define SHELL_ECHO 1

#define SHELL_LINE_LEN 50
#define SHELL_MAX_ARGS 4

enum {
    LINE_OK = 0,
    LINE_OVERFLOW,
    END_OF_TRANSMISSION
};

// reads a NULL-terminated line
static int read_line(SerialDevice *dev, char *line, size_t len)
{
    char *l = line;
    while (l < line + len - 1) {
        int c = SERIAL_GETC(dev);
        if (c == '\0' || c == 4 || c == SERIAL_EOF || c < 0) { // CTRL+D to exit
            return END_OF_TRANSMISSION;
        }
        if (c == '\r') {
            *l = '\0';
#if SHELL_ECHO
            SERIAL_WRITE(dev, (uint8_t *)"\r\n", 2);
#endif
            return LINE_OK;
        }
        if (c < 20) {
            continue;
        }
        *l++ = c;
#if SHELL_ECHO
        SERIAL_PUTC(dev, c);
#endif
    }
    *l = '\0';
    return LINE_OVERFLOW;
}

// extract the arguments from a line, returns argc
// returns negative if too many arguments
static int get_args(char *line, char **argv, size_t max_args)
{
    int argc = 0;
    char *l = line;
    char *argp = NULL;
    while (argc <= max_args) {
        while (*l == ' ') { // skip whitespace
            l++;
        }
        argp = l;
        while (*l != ' ' && *l != '\0') { // scan argument
            l++;
        };
        if (argp < l) {
            argv[argc] = argp;
            argc++;
        }
        if (*l == '\0') {
            return argc;
        } else {
            *l = '\0';
            l++;
        }
    }
    return -1;
}

void shell(const struct shell_commands *commands, SerialDevice *dev)
{
    char line[SHELL_LINE_LEN];
    char *argv[SHELL_MAX_ARGS];
    while (1) {
        serial_printf(dev, SHELL_PROMPT);
        int ret = read_line(dev, line, sizeof(line));
        if (ret == END_OF_TRANSMISSION) {
            serial_printf(dev, "\r\nexit\r\n");
            return;
        }
        if (ret == LINE_OVERFLOW) {
            serial_printf(dev, "line too long\r\n");
            continue;
        }
        if (ret != LINE_OK) {
            continue;
        }
        int argc = get_args(line, argv, SHELL_MAX_ARGS);
        if (argc < 0) {
            serial_printf(dev, "too many arguments\r\n");
            continue;
        } else if (argc == 0) {
            continue;
        }
        const struct shell_commands *c = commands;
        while (c->name != NULL && c->function != NULL) {
            if (strcmp(c->name, argv[0]) == 0) {
                c->function(dev, argc - 1, &argv[1]);
                break;
            }
            c++;
        }
        if (c->name == NULL || c->function == NULL) {
            serial_printf(dev, "%s ?\r\n", line); // command not found
        }
    }
}

