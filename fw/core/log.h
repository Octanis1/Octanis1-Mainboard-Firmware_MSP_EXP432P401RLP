/*
 *  File: log.h
 *  Description: High-level module for data logging
 *  Author: Michael and Eloi
 */

#include "../../Board.h"
#include "../lib/cmp/cmp.h"
#include "../lib/cmp_mem_access/cmp_mem_access.h"
#include <stdint.h>

#define LOGGING_BUFFER_SIZE 42

/* The following objects are created statically. See app.cfg */
// extern Semaphore_Handle logging_sem; <---since we don't want to
// log thing right after they are pushed to the queue, for performance
// reasons, there's no need for semaphore, since TIRTOS will only have
// one task at a time accessing the Queue.
extern Queue_Handle logging_queue;

/* Moved the struct in the .h, not sure if necessary */
/*Biggest block is currently weather, with 
 * 6*4 bytes int (6*5)
 * a 3 char ID name (gps, imu, wea, ...) (+5)
 * 4 bytes timestamp (+5)
 * a crc8 (+2)
 * So the buffer need to be at least 42-bytes long*/
struct logger {
    cmp_ctx_t ctx;
    cmp_mem_access_t cma;
    uint8_t data [LOGGING_BUFFER_SIZE];
};

//Given a name (gps, wea, etc.), serialize the current data into a buffer
void logger_make(const char *name);

//Function to serialize the data into a buffer using MessagePack
void cmp_logger_get(const char *name, struct logger *l);
void logging_gps_serialize(struct logger *l);
void logging_imu_serialize(struct logger *l);
void logging_weather_serialize(struct logger *l);

/*Respectively calculate crc and push the message
 * or pop the message and check crc
 * return true if operation succeed, false if fail*/
uint8_t logger_finish(struct logger *l);
uint8_t logger_pop (struct logger *l);
