#include <stdint.h>
#include <stddef.h>
#include "flash_defines.h"
#include "flash.h"

// defined in hal/flash_spi.c
void flash_spi_select(void);
void flash_spi_unselect(void);
int flash_spi_send(const void *txbuf, size_t len);
int flash_spi_receive(void *rxbuf, size_t len);

int flash_write_enable(void)
{
    return 0;
}

int flash_write_disable(void)
{
    return 0;
}

int flash_read(uint32_t addr, void *buf, size_t len)
{
    // FLASH_ASSERT(addr < (1<<24));
    int ret;
    uint8_t cmd[4] = {FLASH_READ, (addr>>16) & 0xff,
                      (addr>>8) & 0xff, addr & 0xff};
    flash_spi_select();
    ret = flash_spi_send(cmd, sizeof(cmd));
    if (ret == 0) {
        ret = flash_spi_receive(buf, len);
    }
    flash_spi_unselect();
    return ret;
}

#if FLASH_TEST
// defined externally for unit testing.
int flash_page_program(uint32_t addr, const void *buf, size_t len);
#else
static int flash_page_program(uint32_t addr, const void *buf, size_t len)
{
    // FLASH_ASSERT(addr < (1<<24));
    // FLASH_ASSERT(addr % FLASH_PAGE_SIZE + len <= FLASH_PAGE_SIZE);
    int ret;
    uint8_t cmd[4] = {FLASH_PP, (addr>>16) & 0xff,
                      (addr>>8) & 0xff, addr & 0xff};
    flash_spi_select();
    ret = flash_spi_send(cmd, sizeof(cmd));
    if (ret == 0) {
        ret = flash_spi_send(buf, len);
    }
    flash_spi_unselect();
    return ret;
}
#endif // FLASH_TEST

int flash_write(uint32_t addr, const void *buf, size_t len)
{
    // todo
    flash_page_program(addr, buf, len);
    return 0;
}

int flash_sector_erase(uint32_t addr)
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
