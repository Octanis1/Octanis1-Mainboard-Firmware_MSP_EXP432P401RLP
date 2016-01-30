#include <stdint.h>
#include <stddef.h>
#include "flash.h"

// defined int hal/flash_spi.c
void flash_spi_select(void);
void flash_spi_unselect(void);
int flash_spi_send(const void *txbuf, size_t len);
int flash_spi_receive(void *rxbuf, size_t len);


int flash_read(uint32_t addr, void *buf, size_t len)
{
    return 0;
}

int flash_write(uint32_t addr, const void *buf, size_t len)
{
    return 0;
}

int flash_page_erase(uint32_t addr)
{
    return 0;
}

int flash_block_erase(uint32_t addr)
{
    return 0;
}

int flash_chip_erase(void)
{
    return 0;
}
