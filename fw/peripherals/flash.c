#include <stdint.h>
#include <stddef.h>
#include "flash_defines.h"
#include "flash.h"

/* Timing (max values): from Spansion S25FL127S (Datasheet Rev06, Chapter 10.10)
    - Page Programming: 1480us (512 bytes)
    - Page Programming: 1185us (256 bytes)
    - Sector Erase Time: 780ms (64-kB / 4-kB physical sectors)
    - Sector Erase Time: 12600ms (64 kB Top/Bottom: logical sector = 16 x 4-kB physical sectors)
    - Sector Erase Time: 3120ms (256-kB logical sectors = 4 x 64-kB physical sectors)
    - Bulk Erase Time: 210s
*/

#define FLASH_PAGE_PROGRAM_TIMEOUT_MS   (2)
#define FLASH_SECTOR_ERASE_TIMEOUT_MS   (780)
#define FLASH_BLOCK_ERASE_TIMEOUT_MS    (12600)
#define FLASH_CHIP_ERASE_TIMEOUT_MS     (210000)

// defined in hal/flash_spi.c
void flash_spi_select(void);
void flash_spi_unselect(void);
int flash_spi_send(const void *txbuf, size_t len);
int flash_spi_receive(void *rxbuf, size_t len);
void flash_os_sleep_ms(uint32_t ms);

static int flash_read_status(uint8_t *sr)
{
    int ret;
    uint8_t cmd = FLASH_RDSR;
    flash_spi_select();
    ret = flash_spi_send(&cmd, 1);
    if (ret == 0) {
        ret = flash_spi_receive(sr, 1);
    }
    flash_spi_unselect();
    return ret;
}

// wait until the flash program/erase operation is finished
static int flash_wait_until_done(uint32_t timeout_ms)
{
    int ret;
    while (timeout_ms-- > 0) {
        uint8_t status;
        ret = flash_read_status(&status);
        if (ret != 0 || (status & STATUS_BUSY) == 0) {
            return ret;
        }
        flash_os_sleep_ms(1);
    }
    return -1;
}

static int flash_cmd(uint8_t cmd)
{
    return flash_spi_send(&cmd, sizeof(cmd));
}

static int flash_cmd_w_addr(uint8_t cmd, uint32_t addr)
{
    if (addr >= (1<<24)) {
        return -1;
    }
    uint8_t buf[4] = {cmd, (addr>>16) & 0xff, (addr>>8) & 0xff, addr & 0xff};
    return flash_spi_send(buf, sizeof(buf));
}

int flash_write_enable(void)
{
    int ret;
    flash_spi_select();
    ret = flash_cmd(FLASH_WREN);
    flash_spi_unselect();
    return ret;
}

int flash_write_disable(void)
{
    int ret;
    flash_spi_select();
    ret = flash_cmd(FLASH_WRDI);
    flash_spi_unselect();
    return ret;
}

int flash_id_read(uint8_t *id)
{
    int ret;
    flash_spi_select();
    ret = flash_cmd(FLASH_JEDEC_ID);
    if (ret == 0) {
        ret = flash_spi_receive(id, 3);
    }
    flash_spi_unselect();
    return ret;
}

int flash_read(uint32_t addr, void *buf, size_t len)
{
    int ret;
    flash_spi_select();
    ret = flash_cmd_w_addr(FLASH_READ, addr);
    if (ret == 0) {
        ret = flash_spi_receive(buf, len);
    }
    flash_spi_unselect();
    return ret;
}

#if defined(FLASH_TEST)
// defined externally for unit testing
int flash_page_program(uint32_t addr, const void *buf, size_t len);
#else

static int flash_page_program(uint32_t addr, const void *buf, size_t len)
{
    if (addr % FLASH_PAGE_SIZE + len > FLASH_PAGE_SIZE) {
        return -1;
    }
    int ret;
    flash_spi_select();
    ret = flash_cmd_w_addr(FLASH_PP, addr);
    if (ret == 0) {
        ret = flash_spi_send(buf, len);
    }
    flash_spi_unselect();
    if (ret == 0) {
        ret = flash_wait_until_done(FLASH_PAGE_PROGRAM_TIMEOUT_MS);
    }
    return ret;
}

#endif // FLASH_TEST

int flash_write(uint32_t addr, const void *buf, size_t len)
{
    int ret;
    size_t align = FLASH_PAGE_SIZE - addr % FLASH_PAGE_SIZE;
    align = align < len ? align : len;
    const uint8_t *p = (const uint8_t *) buf;
    ret = flash_page_program(addr, p, align);
    if (ret == 0) {
        size_t pos = align;
        while (pos < len) {
            size_t n = len - pos < FLASH_PAGE_SIZE ? len - pos : FLASH_PAGE_SIZE;
            ret = flash_page_program(addr + pos, p + pos, n);
            if (ret != 0) {
                break;
            }
            pos += n;
        }
    }
    return ret;
}

int flash_sector_erase(uint32_t addr)
{
    int ret;
    flash_spi_select();
    ret = flash_cmd_w_addr(FLASH_SE, addr);
    flash_spi_unselect();
    if (ret == 0) {
        ret = flash_wait_until_done(FLASH_SECTOR_ERASE_TIMEOUT_MS);
    }
    return ret;
}

int flash_block_erase(uint32_t addr)
{
    int ret;
    flash_spi_select();
    ret = flash_cmd_w_addr(FLASH_BE, addr);
    flash_spi_unselect();
    if (ret == 0) {
        ret = flash_wait_until_done(FLASH_BLOCK_ERASE_TIMEOUT_MS);
    }
    return ret;
}

int flash_chip_erase(void)
{
    int ret;
    flash_spi_select();
    ret = flash_cmd(FLASH_CE);
    flash_spi_unselect();
    if (ret == 0) {
        ret = flash_wait_until_done(FLASH_CHIP_ERASE_TIMEOUT_MS);
    }
    return ret;
}
