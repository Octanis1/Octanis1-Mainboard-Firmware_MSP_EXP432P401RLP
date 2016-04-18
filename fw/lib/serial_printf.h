#ifndef SERIAL_PRINTF_H
#define SERIAL_PRINTF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "serial.h"

int serial_printf(SerialDevice *dev, const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif /* SERIAL_PRINTF_H */
