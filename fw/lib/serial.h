#ifndef SERIAL_H
#define SERIAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

#define SERIAL_EOF      -1

struct serial_function_table {
    size_t (*write_fn)(void *dev, const uint8_t *data, size_t n);
    size_t (*read_fn)(void *dev, uint8_t *data, size_t n);
    int (*putc_fn)(void *dev, uint8_t c);
    int (*getc_fn)(void *dev);
};

typedef struct {
    const struct serial_function_table *fntab;
} SerialDevice;

/* Writes n bytes from data to dev, returns the number of bytes written */
size_t serial_write(SerialDevice *dev, const uint8_t *data, size_t n);
/* Reads n bytes from dev into data, returns the number of bytes read */
size_t serial_read(SerialDevice *dev, uint8_t *data, size_t n);
/* Writes a byte to dev, returns 0 on success */
int serial_putc(SerialDevice *dev, uint8_t c);
/* Reads a byte from dev, returns SERIAL_EOF if at end of file */
int serial_getc(SerialDevice *dev);

/* Macro versions of the same functions above. */
#define SERIAL_WRITE(dev, data, n) ((dev)->fntab->write_fn(dev, data, n))
#define SERIAL_READ(dev, data, n) ((dev)->fntab->read_fn(dev, data, n))
#define SERIAL_PUTC(dev, c) ((dev)->fntab->putc_fn(dev, c))
#define SERIAL_GETC(dev) ((dev)->fntab->getc_fn(dev))

#ifdef __cplusplus
}
#endif

#endif /* SERIAL_H */
