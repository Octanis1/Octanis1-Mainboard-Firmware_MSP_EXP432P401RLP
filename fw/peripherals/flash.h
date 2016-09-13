#ifndef FLASH_H
#define FLASH_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// read the 3-byte flash JEDEC-ID, id[0] = manufacturer, id[1-2] = device type
int flash_id_read(uint8_t *id);

int flash_read_registers(uint8_t *buf);
int flash_write_registers(uint8_t *sr1, uint8_t *cr, uint8_t *sr2);
int flash_read_status(uint8_t *buf);

// reads data from flash
int flash_read(uint32_t addr, void *buf, size_t len);

// write data to flash
int flash_write(uint32_t addr, const void *buf, size_t len);

// erase a 4 KByte sector at address addr
// Note: does not work for Spansion S25FL127S, use flash_block_erase() instead
int flash_sector_erase(uint32_t addr);

// erase a block (64 KByte for Spansion S25FL127S)
// Note: check out the memory map in the datasheet
int flash_block_erase(uint32_t addr);

// erase the chip
int flash_chip_erase(void);

// init flash chip
int flash_init();

#ifdef __cplusplus
}
#endif

#endif /* FLASH_H */
