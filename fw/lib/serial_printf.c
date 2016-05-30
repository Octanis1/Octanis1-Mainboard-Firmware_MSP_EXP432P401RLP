#include <stdarg.h>
#include "serial.h"
#include "printf.h"

static void _serial_printf_putc(void *arg, char c)
{
    SERIAL_PUTC((SerialDevice *)arg, (uint8_t)c);
}

int serial_printf(SerialDevice *dev, const char *fmt, ...)
{
    va_list va;
    int n;
    va_start(va, fmt);
    n = tfp_format(dev, _serial_printf_putc, fmt, va);
    va_end(va);
    return n;
}

int serial_vprintf(SerialDevice *dev, const char *fmt, va_list va)
{
    return tfp_format(dev, _serial_printf_putc, fmt, va);
}
