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

bool log_read_entry(uint32_t addr, uint8_t buf[LOG_ENTRY_DATA_LEN], size_t *entry_len, uint32_t *next_entry)
{
    int ret;
    uint8_t header[2];
    logger_lock();
    ret = flash_read(addr, header, 2);
    size_t len = header[0];
    uint8_t crc = header[1];
    if (ret == 0 && len > 0) {
        ret = flash_read(addr + 2, buf, len);
    }
    logger_unlock();
    if (len == 0 || ret != 0 || crc8(0, buf, len) != crc) {
        return false;
    }
    *entry_len = len;
    *next_entry = len + 2;
    return true;
}

// Tells if a flash block must be erased before a flash_write(addr, len) call.
// returns true if flash block at erase_addr must be erased first.
// Note: assumes that len < FLASH_BLOCK_SIZE
LOG_INTERNAL bool _log_flash_erase_addr(uint32_t addr, size_t len, uint32_t *erase_addr)
{
    if (addr % FLASH_BLOCK_SIZE == 0) {
        // at start of new flash block
        *erase_addr = addr;
        return true;
    }
    uint32_t end = (addr + len - 1);
    if (addr % FLASH_BLOCK_SIZE != end % FLASH_BLOCK_SIZE) {
        // write length passes into a new flash block
        *erase_addr = end - end % FLASH_BLOCK_SIZE;
        return true;
    }
    return false;
}

LOG_INTERNAL void _log_flash_write(struct logger *l, void *data, size_t len)
{
    uint32_t erase_addr;
    uint32_t addr = l->flash_write_pos;
    if (_log_flash_erase_addr(addr, len, &erase_addr)) {
        flash_block_erase(erase_addr);
    }
    flash_write(addr, data, len);
    l->flash_write_pos += len;
}

// get the latest position in the backup
// returns true if addr is valid, otherwise assume empty or corrupt backup table
LOG_INTERNAL bool _log_backup_table_pos(uint32_t *addr)
{
    return false;
}

// search end of flash log
// the end is considered found if len consecutive bytes are unwritten (=0xff)
LOG_INTERNAL bool _log_seek_end(uint32_t base_addr, uint32_t *end_addr, uint8_t *buf, size_t len)
{
    uint32_t addr = base_addr;
    while (addr + len < FLASH_SIZE) {
        flash_read(addr, buf, len);
        bool might_be_end = false;
        size_t possible_end_index = 0;
        size_t i;
        for (i = 0; i < len; i++) {
            if (buf[i] == 0xff) {
                if (!might_be_end) {
                    possible_end_index = i;
                }
                might_be_end = true;
            } else {
                might_be_end = false;
            }
        }
        if (might_be_end) {
            if (possible_end_index == 0) {
                *end_addr = addr + possible_end_index;
                return true;
            }
            addr += possible_end_index;
        } else {
            addr += len;
        }
    }
    return false;
}

void log_position_backup(void)
{

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
