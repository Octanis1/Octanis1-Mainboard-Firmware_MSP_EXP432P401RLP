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
#include <string.h>
#include <stdint.h>
#include "../lib/cmp/cmp.h"
#include "../lib/cmp_mem_access/cmp_mem_access.h"

#define TIMESTAMP_TO_MILLISEC 48000
Timestamp_get32()/TIMESTAMP_TO_MILLISEC

#if defined(LOG_TEST)
#define LOG_INTERNAL
#else
#define LOG_INTERNAL static
#endif

extern void logger_lock(void);
extern void logger_unlock(void);
extern uint32_t logger_timestamp_millisec(void);

LOG_INTERNAL struct logger;

LOG_INTERNAL cmp_ctx_t *_log_entry_create(struct logger *l, const char *name)
{
    logger_lock();
    cmp_mem_access_init(&l->ctx, &l->cma, l->buffer[LOG_ENTRY_HEADER_LEN], LOG_ENTRY_DATA_LEN);
    cmp_write_array(&l->ctx, 3);
    cmp_write_uinteger(&l->ctx, );
    cmp_write_str(&l->ctx, name, strlen(name));
    return &l->ctx;
}

LOG_INTERNAL void _log_entry_write_to_flash(struct logger *l)
{
    size_t len = cmp_mem_access_get_pos(l->cma);
    crc = crc8(&l->buffer[LOG_ENTRY_HEADER_LEN], len);
    l->buffer[0] = len;
    l->buffer[1] = crc;
    logger_flash_write(l->buffer, len + LOG_ENTRY_HEADER_LEN);
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
