/*
 *  File: log.h
 *  Description: High-level module for data logging
 *  Author: Michael and Eloi
 */

#include "../../Board.h"
#include "../lib/cmp/cmp.h"
#include "../lib/cmp_mem_access/cmp_mem_access.h"
#include <stdint.h>

/* The following objects are created statically. See app.cfg */
// extern Semaphore_Handle logging_sem; <---since we don't want to
// log thing right after they are pushed to the queue, for performance
// reasons, there's no need for semaphore, since TIRTOS will only have
// one task at a time accessing the Queue.
extern Queue_Handle logging_queue;

/* Moved the struct in the .h, not sure if necessary */
struct logger {
    cmp_ctx_t ctx;
    cmp_mem_access_t cma;
    uint8_t data;
};


struct logger *cmp_logger_get(const char *name);

/*Respectively calculate crc and push the message
 * or pop the message and check crc
 * return true if operation succeed, false if fail*/
uint8_t logger_finish(struct logger *l);
uint8_t logger_pop (struct logger *l);
