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

#define LOG_ENTRY_HEADER_LEN    4 // counter + length byte + crc8
#define LOG_ENTRY_DATA_LEN      1124

/* Moved the struct in the .h, not sure if necessary */
/*Biggest block is currently the mavlink waypoints:
 * 17 bytes of "header"
 * and up to 21*52 = 1092 bytes of items
 * so 1109 bytes*/
struct logger {
    // entry buffer
    cmp_ctx_t ctx;
    cmp_mem_access_t cma;
    uint8_t buffer[LOG_ENTRY_HEADER_LEN + LOG_ENTRY_DATA_LEN];
    // flash state
    uint32_t flash_write_pos; // points at the next empty flash position
    uint32_t mav_write_pos; // points at the next empty mavlink location
    uint32_t backup_pos; // points at the next empty backup position
};


cmp_ctx_t *log_entry_create(const char *name);
void log_entry_write_to_flash(void);
void log_mav_write_to_flash(uint8_t *pos_counter);
void log_mav_overwrite_flash(void);
bool log_read_entry(uint32_t addr, uint8_t buf[LOG_ENTRY_DATA_LEN], size_t *entry_len, uint32_t *next_entry, uint8_t *pos_counter);
bool log_read_last_mav_entry (uint8_t buf[LOG_ENTRY_DATA_LEN], size_t *entry_len, uint32_t *next_entry, uint8_t *pos_counter);
void log_position_backup(void);
bool log_init(void);
void log_reset(void);
uint32_t log_write_pos(void);

#endif /* LOG_H */
