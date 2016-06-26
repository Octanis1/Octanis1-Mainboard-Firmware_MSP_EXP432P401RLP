#include <stdarg.h>
#include "serial.h"
#include "printf.h"
#include "../../MSP_EXP432P401RLP.h"

static void _serial_printf_putc(void *arg, char c)
{
    SERIAL_PUTC((SerialDevice *)arg, (uint8_t)c);
}

int serial_printf(SerialDevice *dev, const char *fmt, ...)
{
#ifndef MAVLINK_ON_UART0_ENABLED
    va_list va;
    int n;
    va_start(va, fmt);
    n = tfp_format(dev, _serial_printf_putc, fmt, va);
    va_end(va);
    return n;
#else
    return 0;
#endif
}

int serial_vprintf(SerialDevice *dev, const char *fmt, va_list va)
{
    return tfp_format(dev, _serial_printf_putc, fmt, va);
}
