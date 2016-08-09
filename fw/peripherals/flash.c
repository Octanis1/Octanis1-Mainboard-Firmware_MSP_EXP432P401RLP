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

static void flash_write_enable(void)
{
    flash_spi_select();
    flash_cmd(FLASH_WREN);
    flash_spi_unselect();
}

static void flash_write_disable(void)
{
    flash_spi_select();
    flash_cmd(FLASH_WRDI);
    flash_spi_unselect();
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

/* NOTICE : in flash_write and flash_read you'll notice that we add a 3*i offset when
 * reading multiple page. This is a magic number to avoid loss of data, flash_write
 * apparently not succeeding in writing the 3*i first entries of its buffer to the
 * begining of the 1+ith page.
 * The reason for the bug is unknow and the programmer choosed to go the easy
 * way after failing at fixing it for more time that he would admit.
 */

//TODO: works with mavlink_log, should test it for standards log.
int flash_read(uint32_t addr, void *buf, size_t len)
{
    int ret = 0;
    size_t pos = 0;

    //Find number of bytes until end of page
    size_t align = FLASH_PAGE_SIZE - (addr % FLASH_PAGE_SIZE);
    align = align < len ? align : len;

    //reduce length of bytes to read if full page is supposed to be read
    if (align >= FLASH_PAGE_SIZE) {
    	align = align / 2;
    }

    //TODO: maybe have to redo spi select/unselect for every read

    flash_spi_select();
    while (pos < len && ret == 0) {

    	ret = flash_cmd_w_addr(FLASH_READ, addr + pos);
    	if (ret == 0) {
			ret = flash_spi_receive(&buf[pos], align);
		}

    	pos += align;
    	align = len - pos < FLASH_PAGE_SIZE / 2 ? len - pos : FLASH_PAGE_SIZE / 2;
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
    int ret;

    if (addr % FLASH_PAGE_SIZE + len > FLASH_PAGE_SIZE) {
        return -1;
    }

    //workaround as library does not allow to flash single page in one go.
    //if more than possible bytes to write, split in two.
    if (len > FLASH_USABLE_LENGTH) {
    	flash_write_enable();
    	flash_spi_select();
    	ret = flash_cmd_w_addr(FLASH_PP, addr);
		if (ret == 0) {
			ret = flash_spi_send(buf, FLASH_USABLE_LENGTH);
		}
		flash_spi_unselect();
		if (ret == 0) {
			ret = flash_wait_until_done(FLASH_PAGE_PROGRAM_TIMEOUT_MS);
		}
    	flash_write_disable();

    	if (ret != 0) {
    		return ret;
    	}

    	addr = addr + FLASH_USABLE_LENGTH;
    	len = len - FLASH_USABLE_LENGTH;
    }

    flash_write_enable();
    flash_spi_select();
    ret = flash_cmd_w_addr(FLASH_PP, addr);
    if (ret == 0) {
        ret = flash_spi_send(buf, len);
    }
    flash_spi_unselect();
    if (ret == 0) {
        ret = flash_wait_until_done(FLASH_PAGE_PROGRAM_TIMEOUT_MS);
    }
    flash_write_disable();
    return ret;
}

#endif // FLASH_TEST

/* NOTICE : in flash_write and flash_read you'll notice that we add a 3*i offset when
 * reading multiple page. This is a magic number to avoid loss of data, flash_write
 * apparently not succeeding in writing the 3*i first entries of its buffer to the
 * begining of the 1+ith page.
 * The reason for the bug is unknow and the programmer choosed to go the easy
 * way after failing at fixing it for more time that he would admit.
 */

//TODO: works with mavlink_log, should test it for standards log.
int flash_write(uint32_t addr, const void *buf, size_t len, FLASH_WRITE_MODE write_mode)
{
    int ret = 0;
    size_t rewrite_beginning = 0;
    size_t rewrite_end = 0;

    uint8_t write_buffer[FLASH_PAGE_SIZE]; //TODO: verify if buffer not too big for stack
    const uint8_t *p_data = (const uint8_t *) buf;

    size_t pos = 0;
    size_t align = FLASH_PAGE_SIZE - (addr % FLASH_PAGE_SIZE);
    align = align < len ? align : len;

    //need to read page until begin of newly written area
    if (write_mode == FLASH_READ_ERASE_WRITE && addr % FLASH_PAGE_SIZE > 0) {
    	rewrite_beginning = addr % FLASH_PAGE_SIZE;
    	ret = flash_read(addr  - (addr % FLASH_PAGE_SIZE), write_buffer, rewrite_beginning);

    }

    while (pos < len && ret == 0) {

		//need to read page from end of newly written area //TODO: verify following calculation
    	if (write_mode == FLASH_READ_ERASE_WRITE && (addr + pos + align) % FLASH_PAGE_SIZE > 0) {
    		rewrite_end = FLASH_PAGE_SIZE - ((addr + pos + align) % FLASH_PAGE_SIZE);
			ret = flash_read(addr + pos + align, &(write_buffer[(addr + pos + align) % FLASH_PAGE_SIZE]), FLASH_PAGE_SIZE - ((addr + pos + align) % FLASH_PAGE_SIZE));
		}

    	//erase page (sector) before writing new data
    	if (ret == 0 && (write_mode == FLASH_READ_ERASE_WRITE || write_mode == FLASH_ERASE_WRITE)) {
    		ret = flash_sector_erase(addr + pos);
    	}

    	//write old data back if required
    	if (ret == 0 && rewrite_beginning > 0) {
    		ret = flash_page_program(addr - (addr % FLASH_PAGE_SIZE), write_buffer, rewrite_beginning);
    		rewrite_beginning = 0;
    	}
    	if (ret == 0 && rewrite_end > 0) {
    		ret = flash_page_program(addr + pos + align, &(write_buffer[(addr + pos + align) % FLASH_PAGE_SIZE]), rewrite_end);
    		rewrite_end = 0;
		}

    	//write new data
    	if (ret == 0) {
    		ret = flash_page_program(addr + pos, &(p_data[pos]), align);
    	}

    	//update pointers and counters
    	pos += align;
		align = len - pos < FLASH_PAGE_SIZE ? len - pos : FLASH_PAGE_SIZE;
    }
    return ret;
}

int flash_sector_erase(uint32_t addr)
{
    int ret;
    flash_write_enable();
    flash_spi_select();
    ret = flash_cmd_w_addr(FLASH_SE, addr);
    flash_spi_unselect();
    if (ret == 0) {
        ret = flash_wait_until_done(FLASH_SECTOR_ERASE_TIMEOUT_MS);
    }
    flash_write_disable();
    return ret;
}

int flash_block_erase(uint32_t addr)
{
    int ret;
    flash_write_enable();
    flash_spi_select();
    ret = flash_cmd_w_addr(FLASH_BE, addr);
    flash_spi_unselect();
    if (ret == 0) {
        ret = flash_wait_until_done(FLASH_BLOCK_ERASE_TIMEOUT_MS);
    }
    flash_write_disable();
    return ret;
}

int flash_chip_erase(void)
{
    int ret;
    flash_write_enable();
    flash_spi_select();
    ret = flash_cmd(FLASH_CE);
    flash_spi_unselect();
    if (ret == 0) {
        ret = flash_wait_until_done(FLASH_CHIP_ERASE_TIMEOUT_MS);
    }
    flash_write_disable();
    return ret;
}
