#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "flash_defines.h"
#include "flash.h"
#include "hal/spi_helper.h"

#include "../../Board.h"

/* Timing (max values): from Spansion S25FL127S (Datasheet Rev06, Chapter 10.10)
    - Page Programming: 1480us (512 bytes)
    - Page Programming: 1185us (256 bytes)
    - Sector Erase Time: 780ms (64-kB / 4-kB physical sectors)
    - Sector Erase Time: 12600ms (64 kB Top/Bottom: logical sector = 16 x 4-kB physical sectors)
    - Sector Erase Time: 3120ms (256-kB logical sectors = 4 x 64-kB physical sectors)
    - Bulk Erase Time: 210s
*/

#define FLASH_PAGE_PROGRAM_TIMEOUT_MS   (2)
#define FLASH_REGISTER_PROGRAM_TIMEOUT_MS   (200)
#define FLASH_BLOCK_64KB_ERASE_TIMEOUT_MS   (780)
#define FLASH_BLOCK1_64KB_ERASE_TIMEOUT_MS  (12600)
#define FLASH_BLOCK_256KB_ERASE_TIMEOUT_MS  (3120)
#define FLASH_CHIP_ERASE_TIMEOUT_MS     (210000)

// defined in hal/flash_spi.c
void flash_spi_select(void);
void flash_spi_unselect(void);
int flash_spi_send(const void *txbuf, size_t len);
int flash_spi_receive(void *rxbuf, size_t len);
void flash_os_sleep_ms(uint32_t ms);

static int flash_initialized = 0;

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

int flash_read_status(uint8_t *sr)
{
    int ret;
    uint8_t cmd = FLASH_RDSR1;
    flash_spi_select();
    ret = flash_spi_send(&cmd, 1);
    if (ret == 0) {
        ret = flash_spi_receive(sr, 1);
    }
    flash_spi_unselect();
    return ret;
}

static int flash_read_status2(uint8_t *sr)
{
    int ret;
    uint8_t cmd = FLASH_RDSR2;
    flash_spi_select();
    ret = flash_spi_send(&cmd, 1);
    if (ret == 0) {
        ret = flash_spi_receive(sr, 1);
    }
    flash_spi_unselect();
    return ret;
}

static int flash_read_config(uint8_t *cr)
{
    int ret;
    uint8_t cmd = FLASH_RDCR;
    flash_spi_select();
    ret = flash_spi_send(&cmd, 1);
    if (ret == 0) {
        ret = flash_spi_receive(cr, 1);
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

int flash_write_registers(uint8_t *sr1, uint8_t *cr, uint8_t *sr2)
{
	int ret;
	int ret2;
	uint8_t cmd = FLASH_WRR;
	flash_write_enable();
	flash_spi_select();
	ret = flash_spi_send(&cmd, 1);
	//write one config register after the other, null pointer stops process
	if (ret == 0 && sr1) {
		ret = flash_spi_send(sr1, 1);
		if (ret == 0 && cr) {
			ret = flash_spi_send(cr, 1);
			if (ret == 0 && sr2) {
				ret = flash_spi_send(sr2, 1);
			}
		}
	}
	//this is required to start the procedure to write the registers.
	flash_spi_unselect();
	ret2 = flash_wait_until_done(FLASH_REGISTER_PROGRAM_TIMEOUT_MS);
	flash_write_disable();
	return ret == 0 ? ret2 : ret;
}

int flash_id_read(uint8_t *id)
{
    int ret;
    flash_spi_select();
    ret = flash_cmd(FLASH_RDID);
    if (ret == 0) {
        ret = flash_spi_receive(id, 3);
    }
    flash_spi_unselect();
    return ret;
}

int flash_read_registers(uint8_t *buf)
{
	int ret = 0;
	ret = flash_read_status(buf);
	if (ret == 0) {
		ret = flash_read_config(&(buf[1]));
	}
	if (ret == 0) {
		ret = flash_read_status2(&(buf[2]));
	}
	return ret;
}

//TODO: works with mavlink_log, should test it for standards log.
int flash_read(uint32_t addr, void *buf, size_t len)
{
    int ret = 0;
    size_t pos = 0;
    uint8_t *p_buf = (uint8_t*)buf;

    //read is consecutive and independent of page
    size_t align = len < FLASH_MAX_MESSAGE ? len : FLASH_MAX_MESSAGE;

    flash_spi_select();
	ret = flash_cmd_w_addr(FLASH_READ, addr);

    while (pos < len && ret == 0) {
		ret = flash_spi_receive(&p_buf[pos], align);
    	pos += align;
    	align = len - pos < FLASH_MAX_MESSAGE ? len - pos : FLASH_MAX_MESSAGE;
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
    uint8_t *_buf = (uint8_t*)buf;

    if (addr % FLASH_PAGE_SIZE + len > FLASH_PAGE_SIZE) {
        return -1;
    }

    //workaround as library does not allow to flash single page in one go.
    //if more than possible bytes to write, split in two.

	flash_write_enable();
	flash_spi_select();
	ret = flash_cmd_w_addr(FLASH_PP, addr);
	if (len > FLASH_MAX_MESSAGE) {
		if (ret == 0) {
			ret = flash_spi_send(buf, FLASH_MAX_MESSAGE);
		}
		if (ret == 0) {
			ret = flash_spi_send(&(_buf[FLASH_MAX_MESSAGE]), len - FLASH_MAX_MESSAGE);
		}
	} else {
		if (ret == 0) {
			ret = flash_spi_send(buf, len);
		}
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
int flash_write(uint32_t addr, const void *buf, size_t len)
{
    int ret = 0;

    const uint8_t *p_data = (const uint8_t *) buf;

    size_t pos = 0;
    size_t align = FLASH_PAGE_SIZE - (addr % FLASH_PAGE_SIZE);
    align = align < len ? align : len;

    while (pos < len && ret == 0) {
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

//sector erase (4kB) only works for first 64kB
int flash_sector_erase(uint32_t addr)
{
    int ret;
    if (addr < FLASH_BLOCK_SIZE) {
		flash_write_enable();
		flash_spi_select();
		ret = flash_cmd_w_addr(FLASH_P4E, addr);
		flash_spi_unselect();
		if (ret == 0) {
			ret = flash_wait_until_done(FLASH_BLOCK_64KB_ERASE_TIMEOUT_MS);
		}
		flash_write_disable();
		return ret;
    }
    return -1;
}

//Only valid if using 64kB block size
int flash_block_erase(uint32_t addr)
{
    int ret;
    flash_write_enable();
    flash_spi_select();
    ret = flash_cmd_w_addr(FLASH_SE, addr);
    flash_spi_unselect();
    if (ret == 0) {
    	if (addr < FLASH_BLOCK_SIZE) {
    		ret = flash_wait_until_done(FLASH_BLOCK1_64KB_ERASE_TIMEOUT_MS);
    	} else {
    		ret = flash_wait_until_done(FLASH_BLOCK_64KB_ERASE_TIMEOUT_MS);
    	}
    }
    flash_write_disable();
    return ret;
}

int flash_chip_erase(void)
{
    int ret;
    flash_write_enable();
    flash_spi_select();
    ret = flash_cmd(FLASH_BE);
    flash_spi_unselect();
    if (ret == 0) {
        ret = flash_wait_until_done(FLASH_CHIP_ERASE_TIMEOUT_MS);
    }
    flash_write_disable();
    return ret;
}

int flash_init()
{
	int ret;
	uint8_t buf[10];

	//return if flash already initialized
	if (flash_initialized == 1) {
		return 0;
	}

	spi_helper_init_handle();

	//workaround to give flash enough time to power up and get ready
	//Task_sleep(10);

	//verify flash chip ID
	ret = flash_id_read(buf);
	if (ret != 0) {
		return ret;
	}
	const uint8_t flash_id[] = {0x01,0x20,0x18}; // S25FL127S ID
	if (memcmp(buf, flash_id, sizeof(flash_id)) == 0) {
		// flash answers with correct ID
		//serial_printf(cli_stdout, "Flash ID OK\n");
	} else {
		//serial_printf(cli_stdout, "Flash ID ERROR\n");
		return -2;
	}

	//verify if registers are set correctly. Should always be the case except for new flash chip
	ret = flash_read_registers(buf);
	if (ret != 0) {
		return ret;
	}
	if (buf[0] != 0x00 || buf[1] != 0xC0 || buf[2] != 0x00)
	{
		uint8_t sr1 = 0x00;
		uint8_t cr = 0xC0;
		uint8_t sr2 = 0x00;

		if (flash_write_registers(&sr1, &cr, &sr2) != 0) {
			//serial_printf(cli_stdout, "Writing flash register ERROR\n");
			return -3;
		} else {
			//serial_printf(cli_stdout, "Writing flash register OK\n");
		}
	}
	flash_initialized = 1;
	return 0;
}
