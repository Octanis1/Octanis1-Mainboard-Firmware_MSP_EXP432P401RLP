#ifndef FLASH_H
#define FLASH_H

#include <stdint.h>
#include <stddef.h>

#define FLASH_PAGE_SIZE 256

#ifdef __cplusplus
extern "C" {
#endif

int flash_read(uint32_t addr, void *buf, size_t len);

int flash_write_enable(void);

int flash_write_disable(void);

int flash_write(uint32_t addr, const void *buf, size_t len);

int flash_page_erase(uint32_t addr);

int flash_block_erase(uint32_t addr);

int flash_chip_erase(void);

#ifdef __cplusplus
}
#endif

#endif /* FLASH_H */
