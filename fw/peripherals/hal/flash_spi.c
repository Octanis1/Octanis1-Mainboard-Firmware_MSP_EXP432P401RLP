#include <stddef.h>
#include <stdint.h>
#include "spi_helper.h"
#include "../../../Board.h"

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>

void flash_spi_lock(void) {
	Semaphore_pend(semFlash, BIOS_WAIT_FOREVER);
}

void flash_spi_unlock(void) {
	Semaphore_post(semFlash);
}

void flash_spi_select(void)
{
	flash_spi_lock();
	GPIO_write(Board_Flash_CS, 0);
}

void flash_spi_unselect(void)
{
	GPIO_write(Board_Flash_CS, 1);
	flash_spi_unlock();
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
    Task_sleep(ms);
    return;
}
