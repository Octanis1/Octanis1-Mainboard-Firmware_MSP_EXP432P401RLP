/*
 *  File: log.c
 *  Description: High-level module for data logging
 *  Author: Michael and Eloi
 */
#include "log.h"

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
- block allocator <---- currently replaced by the mailboc function. Can hold 20 32-bytes message.
- crc8 function
- serialization functions for different sensor values
*/

struct logger *cmp_logger_get(const char *name)
{
    struct logger *l = (struct logger *) logging_buffer_alloc();

    size_t size = LOGGING_BUFFER_SIZE - ((uintptr_t)&l->data + (uintptr_t)l);
    cmp_mem_access_init(&l->cma, &l->ctx, &l->data, size);

    //Get number of clock tick since beginning
    //frequency : 48MHz
    uint32_t t = Timestamp_get32();
    t = t/48000; //converted in ms here

    cmp_write_uinteger(&l->ctx, t);
    cmp_write_str(&l->ctx, name);
    return l;
}

uint8_t logger_finish(struct logger *l)
{
    logging_passing_pointer log;
    log.l_point = l;

    // calculate crc

    // Save the logging info (whole struct) into the mailbox
    // return true is saving succeeded, fase if mailbox full
    return Mailbox_post(logging_mailbox, l, BIOS_NO_WAIT);
}


uint8_t logger_pop(struct logger *l)
{
    uint8_t ret = Mailbox_pend(logging_mailbox, l, BIOS_NO_WAIT);

    // check crc
    return ret;
}
