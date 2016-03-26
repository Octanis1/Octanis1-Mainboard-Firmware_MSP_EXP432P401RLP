/*
 *  File: log.h
 *  Description: High-level module for data logging
 *  Author: Michael and Eloi
 */

//#include "../../Board.h"
#include "../lib/cmp/cmp.h"
#include "../lib/cmp_mem_access/cmp_mem_access.h"
#include <stdint.h>
//mock fun for testing
#include "../tests/log_mock_fun.h"

#define LOGGING_BUFFER_SIZE 50
#define GPS 0
#define IMU 1
#define WEATHER 2

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
    char data [LOGGING_BUFFER_SIZE];
};

/* This struct is used to pass
 * the decoded info of a MessagePack buffer
 * It either contain the weather info, the imu
 * info OR the gps info, depending of the value of
 * data_type. */
typedef struct log_data_t {
    uint8_t data_type;
    float lat;
    float lon;
    uint8_t fix_qual;
    uint8_t imu_calib;
    uint16_t imu_head;
    uint16_t imu_roll;
    uint16_t imu_pitch;
    unsigned int int_press;
    int int_temp;
    unsigned int int_humi;
    unsigned int ex_press;
    int ex_temp;
    unsigned int ex_humi;
}log_data_t;

//Given a name (gps, wea, etc.), serialize the current data into a buffer
void logger_make(const char *name);

//Function to serialize the data into a buffer using MessagePack
void cmp_logger_get(const char *name, struct logger *l);
void logging_gps_serialize(struct logger *l);
void logging_imu_serialize(struct logger *l);
void logging_weather_serialize(struct logger *l);

//decode a buffer encoded with MessagePack
void logging_parse_buffer (char *buffer, log_data_t * decoded);
void logging_parse_gps (struct logger *l, log_data_t * decoded);
void logging_parse_weather (struct logger *l, log_data_t * decoded);
void logging_parse_imu (struct logger *l, log_data_t * decoded);

/*Respectively calculate crc and push the message
 * or pop the message and check crc
 * return true if operation succeed, false if fail*/
uint8_t logger_finish(struct logger *l);
uint8_t logger_pop (struct logger *l);
