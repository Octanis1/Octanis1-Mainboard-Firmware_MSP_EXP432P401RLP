#ifndef SHELL_H
#define SHELL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "serial.h"

/* Shell command function table.
 * name: string containing the command name.
 * function: the function to be called for the corresponding command.
 * Note: The table must be NULL-terminated (name and function set to NULL).
 */
struct shell_commands {
    const char *name;
    void (*function)(SerialDevice *io, int argc, char *argv[]);
};

void shell(const struct shell_commands *commands, SerialDevice *dev);

#ifdef __cplusplus
}
#endif

#endif /* SHELL_H */
