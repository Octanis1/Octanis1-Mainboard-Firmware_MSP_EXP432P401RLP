#ifndef LOG_INTERNAL_H
#define LOG_INTERNAL_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#if defined(LOG_TEST)
#define LOG_INTERNAL
#define FLASH_BLOCK_SIZE    32
#define FLASH_SIZE          (16*FLASH_BLOCK_SIZE)
#else
#define LOG_INTERNAL static
#define FLASH_BLOCK_SIZE    0x10000  // 64K
#define FLASH_SIZE          0x1000000 // 16M
#endif

LOG_INTERNAL void logger_lock(void);
LOG_INTERNAL void logger_unlock(void);
LOG_INTERNAL uint32_t logger_timestamp_sec(void);
LOG_INTERNAL cmp_ctx_t *_log_entry_create(struct logger *l, const char *name);
LOG_INTERNAL void _log_entry_write_to_flash(struct logger *l);
LOG_INTERNAL bool _log_flash_erase_addr(uint32_t addr, size_t len, uint32_t *erase_addr);
LOG_INTERNAL void _log_flash_write(uint32_t addr, void *data, size_t len);
LOG_INTERNAL bool _log_backup_table_get_last(uint32_t *addr);
LOG_INTERNAL bool _log_seek_end(uint32_t base_addr, uint32_t *end_addr, uint8_t *buf, size_t len);
LOG_INTERNAL void _log_position_backup(struct logger *l);

#endif /* LOG_INTERNAL_H */
