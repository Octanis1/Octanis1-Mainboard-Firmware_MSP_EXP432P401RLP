#include <stddef.h>
#include <stdint.h>
#include "spi_helper.h"
#include "../../../Board.h"

void flash_spi_select(void)
{
	GPIO_write(Board_Flash_CS, 0);
}

void flash_spi_unselect(void)
{
	GPIO_write(Board_Flash_CS, 1);
}

int flash_spi_send(const void *txbuf, size_t len)
{
    return spi_helper_transfer(len, (uint8_t*)txbuf, NULL, 0);
}

int flash_spi_receive(void *rxbuf, size_t len)
{
	return spi_helper_transfer(len, NULL, (uint8_t*)rxbuf, 0);
}

void flash_os_sleep_ms(uint32_t ms)
{
    (void)ms;
    return;
}
