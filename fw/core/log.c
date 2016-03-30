/*
 *  File: log.c
 *  Description: High-level module for data logging
 *  Author: Michael and Eloi
 */

/*
Logging format:
- 1byte: length = n
- 1byte: crc8
- nbytes: log data

The log data is MessagePack encoded, http://msgpack.org/index.html
like the following structure:
- timestamp: unsigned integer
- string: data type
- data: MessagePack encoded data structure

CHANGE:
simplify logger: don't use a logging thread, use mutex instead (logger_lock(), logger_unlock())

todo:
- unit tests
- finish/fix serialization functions
*/

#include "log.h"
#include "log_internal.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "../lib/cmp/cmp.h"
#include "../lib/cmp_mem_access/cmp_mem_access.h"
#include "../lib/crc8.h"
#include "../peripherals/flash.h"

#if !defined(LOG_TEST) // Production code
// todo: add mutex if logger needs to be threadsafe
LOG_INTERNAL void logger_lock(void){}
LOG_INTERNAL void logger_unlock(void){}

#include <ti/sysbios/hal/Seconds.h>
LOG_INTERNAL uint32_t logger_timestamp_sec(void)
{
    return Seconds_get();
}
#endif // LOG_TEST

LOG_INTERNAL struct logger logger;

LOG_INTERNAL cmp_ctx_t *_log_entry_create(struct logger *l, const char *name)
{
    logger_lock();
    cmp_mem_access_init(&l->ctx, &l->cma, &l->buffer[LOG_ENTRY_HEADER_LEN], LOG_ENTRY_DATA_LEN);
    cmp_write_array(&l->ctx, 3);
    cmp_write_uinteger(&l->ctx, logger_timestamp_sec());
    cmp_write_str(&l->ctx, name, strlen(name));
    return &l->ctx;
}

LOG_INTERNAL void _log_entry_write_to_flash(struct logger *l)
{
    size_t len = cmp_mem_access_get_pos(&l->cma);
    uint8_t crc = crc8(0, (unsigned char *)&l->buffer[LOG_ENTRY_HEADER_LEN], len);
    l->buffer[0] = len;
    l->buffer[1] = crc;
    _log_flash_write(l, l->buffer, len + LOG_ENTRY_HEADER_LEN);
    logger_unlock();
}

cmp_ctx_t *log_entry_create(const char *name)
{
    return _log_entry_create(&logger, name);
}

void log_entry_write_to_flash(void)
{
    _log_entry_write_to_flash(&logger);
}

bool log_read_entry(uint32_t addr, uint8_t buf[LOG_ENTRY_DATA_LEN], size_t *entry_len)
{
    return false;
}

bool log_read_entry_cmp_reader(uint32_t addr, cmp_ctx_t **ctx, char *name, uint32_t *timestamp)
{
    // todo
    return false;
}

// assumes that len < FLASH_BLOCK_SIZE
// returns true if flash block at erase_addr must be erased first.
LOG_INTERNAL bool _log_flash_erase_addr(uint32_t addr, size_t len, uint32_t *erase_addr)
{
    return false;
}

LOG_INTERNAL void _log_flash_write(struct logger *l, void *data, size_t len)
{
    // todo: erase block if necessary
    uint32_t addr = l->flash_write_pos;
    flash_write(addr, data, len);
    l->flash_write_pos += len;
}

// get the latest position in the backup
// returns true if addr is valid, otherwise assume empty or corrupt backup table
LOG_INTERNAL bool _log_backup_table_pos(uint32_t *addr)
{
    return false;
}

LOG_INTERNAL bool _log_seek_end(uint32_t base_addr, uint32_t *end_addr, uint8_t *buf, size_t len)
{
    return false;
}

// returns false on error
bool log_init(void)
{
    return false;
}

// erases all previous entry
void log_reset(void)
{
    flash_block_erase(0x00000000);
    log_init();
}
