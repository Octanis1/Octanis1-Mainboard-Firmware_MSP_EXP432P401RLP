#ifndef SERIAL_PRINTF_H
#define SERIAL_PRINTF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdarg.h>
#include "serial.h"

int serial_printf(SerialDevice *dev, const char *fmt, ...);

int serial_vprintf(SerialDevice *dev, const char *fmt, va_list va);

#ifdef __cplusplus
}
#endif

#endif /* SERIAL_PRINTF_H */
