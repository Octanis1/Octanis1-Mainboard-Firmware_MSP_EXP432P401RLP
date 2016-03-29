#ifndef FLASH_H
#define FLASH_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// read the 3-byte flash JEDEC-ID, id[0] = manufacturer, id[1-2] = device type
int flash_id_read(uint8_t *id);

// reads data from flash
int flash_read(uint32_t addr, void *buf, size_t len);

// enable/disable flash write & erase access
int flash_write_enable(void);
int flash_write_disable(void);

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

#ifdef __cplusplus
}
#endif

#endif /* FLASH_H */
