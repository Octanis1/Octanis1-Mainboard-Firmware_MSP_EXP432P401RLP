/*
 *  File: log.h
 *  Description: High-level module for data logging
 *  Author: Michael and Eloi
 */

#ifndef LOG_H
#define LOG_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "../lib/cmp/cmp.h"
#include "../lib/cmp_mem_access/cmp_mem_access.h"

#define LOG_ENTRY_HEADER_LEN    2 // length byte + crc8
#define LOG_ENTRY_DATA_LEN      255

/* Moved the struct in the .h, not sure if necessary */
/*Biggest block is currently weather, with
 * 6*4 bytes int (6*5)
 * a 3 char ID name (gps, imu, wea, ...) (+5)
 * 4 bytes timestamp (+5)
 * a crc8 (+2)
 * So the buffer need to be at least 42-bytes long*/
struct logger {
    // entry buffer
    cmp_ctx_t ctx;
    cmp_mem_access_t cma;
    char buffer[LOG_ENTRY_HEADER_LEN + LOG_ENTRY_DATA_LEN];
    // flash state
    uint32_t flash_write_pos; // points at the next empty flash position
    uint32_t backup_pos; // points at the last valid backup position
    bool ready;
};


cmp_ctx_t *log_entry_create(const char *name);
void log_entry_write_to_flash(void);
bool log_read_entry(uint32_t addr, uint8_t buf[LOG_ENTRY_DATA_LEN], size_t *entry_len, uint32_t *next_entry);
bool log_read_entry_cmp_reader(uint32_t addr, cmp_ctx_t **ctx, char *name, uint32_t *timestamp, uint32_t *next_entry);
bool log_init(void);
void log_reset(void);

#endif /* LOG_H */
