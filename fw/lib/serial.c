#include "serial.h"

size_t serial_write(SerialDevice *dev, const uint8_t *data, size_t n)
{
    return dev->fntab->write_fn(dev, data, n);
}

size_t serial_read(SerialDevice *dev, uint8_t *data, size_t n)
{
    return dev->fntab->read_fn(dev, data, n);
}

int serial_putc(SerialDevice *dev, uint8_t c)
{
    return dev->fntab->putc_fn(dev, c);
}

int serial_getc(SerialDevice *dev)
{
    return dev->fntab->getc_fn(dev);
}
