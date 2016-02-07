#include <stddef.h>
#include <stdint.h>

void flash_spi_select(void)
{

}

void flash_spi_unselect(void)
{

}

int flash_spi_send(const void *txbuf, size_t len)
{
    return 0;
}

int flash_spi_receive(void *rxbuf, size_t len)
{
    return 0;
}

void flash_os_sleep_ms(uint32_t ms)
{
    (void)ms;
    return;
}
