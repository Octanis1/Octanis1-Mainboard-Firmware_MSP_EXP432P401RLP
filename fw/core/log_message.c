#include "cli.h"
#include "log_message.h"
#include <serial_printf.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/hal/Seconds.h>

static void write_entry_format(const char *loglevel)
{
    Task_Handle task = Task_self();
    const char *thread_name = Task_Handle_name(task);
    if (thread_name != NULL) {
        thread_name = "";
    }

    uint32_t s = Seconds_get();
    serial_printf(cli_stdout, LOG_COLOR_BLUE "[%u] %s: %s" LOG_COLOR_CLEAR, s,
                  thread_name, loglevel);
}


void log_message(const char *lvl, const char *fmt, ...)
{
    // todo: mutex lock

    va_list args;
    write_entry_format(lvl);

    va_start(args, fmt);
    serial_vprintf(cli_stdout, fmt, args);
    va_end(args);

    serial_printf(cli_stdout, "\n");

    // todo: mutex unlock
}
