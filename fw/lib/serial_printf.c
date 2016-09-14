#include <stdarg.h>
#include "serial.h"
#include "printf.h"
#include "../../MSP_EXP432P401RLP.h"
#include "../peripherals/comm.h"

static void _serial_printf_putc(void *arg, char c)
{
    SERIAL_PUTC((SerialDevice *)arg, (uint8_t)c);
}

static char mav_buffer[50];
static int count;
static void _serial_printf_mav_putc(void *arg, char c)
{
	mav_buffer[count] = c;
	if(count < 49)
		count++;
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
    // print via mavlink message STATUSTEXT #253
    count = 0;
    memset(&mav_buffer, ' ', sizeof(mav_buffer));

    va_list va;
	int n;
	va_start(va, fmt);
	n = tfp_format(dev, _serial_printf_mav_putc, fmt, va);
	va_end(va);

	static COMM_FRAME frame;

	mavlink_msg_statustext_pack(mavlink_system.sysid, MAV_COMP_ID_LOG, &(frame.mavlink_message),
    		MAV_SEVERITY_DEBUG, mav_buffer);

	comm_mavlink_broadcast(&frame);

    return n;
#endif
}

int serial_vprintf(SerialDevice *dev, const char *fmt, va_list va)
{
    return tfp_format(dev, _serial_printf_putc, fmt, va);
}
