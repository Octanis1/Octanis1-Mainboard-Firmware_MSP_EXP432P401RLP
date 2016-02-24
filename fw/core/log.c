/*
 *  File: log.c
 *  Description: High-level module for data logging
 *  Author: Michael and Eloi
 */
#include "log.h"
#include <cmp/cmp.h>
#include <stdint.h>

struct logger {
    cmp_ctx_t ctx;
    cmp_mem_access_t cma;
    uint8_t data;
}

#define LOGGING_BUFFER_SIZE 100

/*

Logging format:
- 1byte: length = n
- 1byte: crc8
- nbytes: log data

the log data is MessagePack encoded, http://msgpack.org/index.html
like the following structure:
- timestamp: unsigned integer
- string: data type
- data: MessagePack encoded data structure

Idea: keep at the beginning of the flash a table to store the last write position

todo:
- get flash driver to work
- flash writer thread (get buffer & write to flash)
- thread message passing of logging buffers (FIFO)
- block allocator
- crc8 function
- serialization functions for different sensor values
*/

struct logger *cmp_logger_get(const char *name)
{
    struct logger *l = (struct logger *) logging_buffer_alloc();

    size_t size = LOGGING_BUFFER_SIZE - ((uintptr_t)&l->data + (uintptr_t)l);
    cmp_mem_access_init(&l->cma, &l->ctx, &l->data, size);

    uint32_t t = timestamp_get();
    cmp_write_uinteger(&l->ctx, t);
    cmp_write_str(&l->ctx, name);
    return l;
}

void logger_finish(struct logger *l)
{
    // calculate crc

    // pass buffer to flash_writer thread
}

