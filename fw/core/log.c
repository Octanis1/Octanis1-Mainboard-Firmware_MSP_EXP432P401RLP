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
    len += LOG_ENTRY_HEADER_LEN;
    _log_flash_write(l->flash_write_pos, l->buffer, len);
    l->flash_write_pos += len;
    logger_unlock();
}

LOG_INTERNAL void _log_mav_write_to_flash(struct logger *l)
{
    size_t len = cmp_mem_access_get_pos(&l->cma);
    uint8_t crc = crc8(0, (unsigned char *)&l->buffer[LOG_ENTRY_HEADER_LEN], len);
    l->buffer[0] = len;
    l->buffer[1] = crc;
    len += LOG_ENTRY_HEADER_LEN;
    _log_flash_write(l->mav_write_pos, l->buffer, len);
    if(l->flash_write_pos == FLASH_BLOCK_SIZE)
    	l->flash_write_pos = 2*FLASH_BLOCK_SIZE;
    else
    	l->flash_write_pos = FLASH_BLOCK_SIZE;

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

void log_mav_write_to_flash(void)
{
    _log_mav_write_to_flash(&logger);
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
        ret = flash_read(addr + LOG_ENTRY_HEADER_LEN, buf, len);
    }
    logger_unlock();
    if (len == 0 || ret != 0 || crc8(0, buf, len) != crc) {
        return false;
    }
    *entry_len = len;
    *next_entry = addr + len + 2;
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
    if (addr - addr % FLASH_BLOCK_SIZE != end - end % FLASH_BLOCK_SIZE) {
        // write length passes into a new flash block
        *erase_addr = end - end % FLASH_BLOCK_SIZE;
        return true;
    }
    return false;
}

LOG_INTERNAL void _log_flash_write(uint32_t addr, void *data, size_t len)
{
    uint32_t erase_addr;
    if (_log_flash_erase_addr(addr, len, &erase_addr)) {
        flash_block_erase(erase_addr);
    }
    flash_write(addr, data, len);
}

// get the latest position in the backup
// returns true if backup_addr is valid, otherwise assume empty or corrupt backup table
LOG_INTERNAL bool _log_backup_table_get_last(uint32_t *backup_addr)
{
    uint32_t addr = 0;
    while(addr < FLASH_BLOCK_SIZE) {
        uint32_t val;
        logger_lock();
        flash_read(addr, (uint8_t *)&val, 4);
        logger_unlock();
        if (val == 0xffffffff) { // empty position
            *backup_addr = addr;
            return true;
        }
        addr += 4;
    }
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

// note: leaves the last entry empty to simplify recovering from a full table
LOG_INTERNAL void _log_position_backup(struct logger *l)
{
    logger_lock();
    uint32_t write_pos = l->flash_write_pos;
    uint32_t bkup_pos = l->backup_pos;
    if (bkup_pos + 4 < FLASH_BLOCK_SIZE) {
        flash_write(bkup_pos, (uint8_t *)&write_pos, 4);
        l->backup_pos += 4;
    } // if backup table is full, just continue without
    logger_unlock();
}

void log_position_backup(void)
{
    _log_position_backup(&logger);
}

LOG_INTERNAL uint32_t _log_get_timestamp(uint32_t addr, uint8_t buf[LOG_ENTRY_DATA_LEN], size_t *entry_len, uint32_t *next_entry)
{
	cmp_mem_access_t cma;
	cmp_ctx_t ctx;

	bool ret = 0;
	uint32_t array_l = 0;
	uint32_t time = 0;

	if (!log_read_entry(addr, buf, entry_len, next_entry))
		return false;

	cmp_mem_access_ro_init(&ctx, &cma, buf, LOG_ENTRY_DATA_LEN);

	ret = cmp_read_array(&ctx, &array_l);
	if (ret == false || array_l != 3)
		return false;

	ret = cmp_read_uint(&ctx, &time);

	return time;
}

LOG_INTERNAL void _log_seek_last_mav_entry()
{
	uint32_t timestamp[2];
	size_t len = 0;
	uint32_t next_entry = 0;
	uint32_t val [2];

	flash_read(FLASH_BLOCK_SIZE, (uint8_t*)&val[0], 4);
	flash_read(2*FLASH_BLOCK_SIZE, (uint8_t*)&val[1], 4);

	if (val[0] == 0xffffffff && val[1] == 0xffffffff)
		logger.mav_write_pos = FLASH_BLOCK_SIZE;
	else if (val[0] == 0xffffffff)
		logger.mav_write_pos = FLASH_BLOCK_SIZE;
	else if (val[1] == 0xffffffff)
		logger.mav_write_pos = 2*FLASH_BLOCK_SIZE;
	else{
		// we have to find the oldest entry
		timestamp[0] = _log_get_timestamp(FLASH_BLOCK_SIZE, logger.buffer, &len, &next_entry);
		timestamp[1] = _log_get_timestamp(2*FLASH_BLOCK_SIZE, logger.buffer, &len, &next_entry);

		if (timestamp[0] > timestamp[1])
			logger.mav_write_pos = 2*FLASH_BLOCK_SIZE;
		else
			logger.mav_write_pos = FLASH_BLOCK_SIZE;
	}
}

//return false if reading log fail or if there's nothing to read
bool log_read_last_mav_entry (uint8_t buf[LOG_ENTRY_DATA_LEN], size_t *entry_len, uint32_t *next_entry)
{
	uint32_t val = 0;

	if(logger.mav_write_pos == FLASH_BLOCK_SIZE){
		flash_read(2*FLASH_BLOCK_SIZE, (uint8_t*)&val, 4);
		if (val == 0xffffffff)
			return false;
		else
			return log_read_entry(2*FLASH_BLOCK_SIZE, buf, entry_len, next_entry);
	}
	else{
		flash_read(FLASH_BLOCK_SIZE, (uint8_t*)&val, 4);
		if (val == 0xffffffff)
			return false;
		else
			return log_read_entry(FLASH_BLOCK_SIZE, buf, entry_len, next_entry);
	}
}

// returns false on error
bool log_init(void)
{

    logger.flash_write_pos = 3*FLASH_BLOCK_SIZE; // fourth block
    logger.backup_pos = 0;
    logger.mav_write_pos = FLASH_BLOCK_SIZE; // second block
    uint32_t bkup_pos = 0;
    if (!_log_backup_table_get_last(&bkup_pos) || bkup_pos == 0) {
        // either empty flash or backup table full (very unlikely)
        // erase & restart logging at beginning of flash
        flash_block_erase(0x00000000);
        _log_position_backup(&logger);
        _log_seek_last_mav_entry();
        return true;
    }

    _log_seek_last_mav_entry();

    uint32_t pos = 0;
    logger_lock();
    flash_read(bkup_pos - 4, &pos, 4);
    logger_unlock();
    if (pos < FLASH_SIZE && pos >= FLASH_BLOCK_SIZE) {
        // if valid position
        if (_log_seek_end(pos, &pos, &logger.buffer[0], sizeof(logger.buffer))) {
            logger.flash_write_pos = pos;
            logger.backup_pos = bkup_pos;
            return true;
        }
    }
    return false;
}

// erases all previous entry
void log_reset(void)
{
    flash_block_erase(0x00000000);
    log_init();
}

uint32_t log_write_pos(void)
{
    return logger.flash_write_pos;
}
