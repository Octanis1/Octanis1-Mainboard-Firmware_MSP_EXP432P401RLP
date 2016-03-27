/*
 *  File: log.c
 *  Description: High-level module for data logging
 *  Author: Michael and Eloi
 */
#include "log.h"
#include <string.h>
#include "../peripherals/weather.h"
#include "../peripherals/gps.h"
#include "--/peripherals/imu.h"
#include <stdint.h>
#include "../lib/cmp/cmp.h"
#include "../lib/cmp_mem_access/cmp_mem_access.h"

#define TIMESTAMP_TO_MILLISEC 48000

#define BIOS_NO_WAIT 0
/*

TODO:test serialization, write crc8

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

void logger_make(const char *name)
{
    struct logger log;

    cmp_logger_get (name, &log);
    logger_finish(&log);
}

void cmp_logger_get(const char *name, struct logger *l)
{
    cmp_mem_access_init(&l->ctx, &l->cma, l->data, sizeof(l->data));
    
    //write name first to identify the entry easily when decoding.
    cmp_write_str(&l->ctx, name, strlen(name));
   
    if (strcmp(name, "gps") == 0)
        logging_gps_serialize(l);
    else if (strcmp(name, "imu") == 0)
        logging_imu_serialize(l);
    else if (strcmp(name, "wea") == 0)
        logging_weather_serialize(l);


    //Get number of clock tick since beginning
    //frequency : 48MHz
    uint32_t t = Timestamp_get32();
    t = t/TIMESTAMP_TO_MILLISEC; //converted in ms here

    cmp_write_u32(&l->ctx, t);
}

void logging_gps_serialize (struct logger *l){
    float lat = gps_get_lat();
    float lon = gps_get_lon();
    uint8_t fix_qual = gps_get_fix_quality();

    cmp_write_float (&l->ctx, lat);
    cmp_write_float (&l->ctx, lon);
    cmp_write_u8 (&l->ctx, fix_qual);
}

void logging_imu_serialize (struct logger *l){
    uint8_t imu_calib = imu_get_calib_status();
    uint16_t imu_head = imu_get_heading();
    uint16_t imu_roll = imu_get_roll();
    uint16_t imu_pitch = imu_get_pitch();

    cmp_write_u8 (&l->ctx, imu_calib);
    cmp_write_u16 (&l->ctx, imu_head);
    cmp_write_u16 (&l->ctx, imu_roll);
    cmp_write_u16 (&l->ctx, imu_pitch);

}
    
void logging_weather_serialize (struct logger *l){

    unsigned int int_press = weather_get_int_press();
    int int_temp = weather_get_int_temp();
    unsigned int int_humi = weather_get_int_humid();
    unsigned int ex_press = weather_get_ext_press();
    int ex_temp = weather_get_ex_temp();
    unsigned int ex_humi = weahter_get_ext_humid();

    cmp_write_u32 (&l->ctx, int_press);
    cmp_write_s32 (&l->ctx, int_temp);
    cmp_write_u32 (&l->ctx, int_humi);
    cmp_write_u32 (&l->ctx, ex_press);
    cmp_write_s32 (&l->ctx, ex_temp);
    cmp_write_u32 (&l->ctx, ex_humi);

}

uint8_t logger_finish(struct logger *l)
{
    //var for testing
  //  Queue_Handle logging_mailbox;
    // calculate crc

    // Save the logging info (whole struct) into the mailbox
    // return true is saving succeeded, fase if mailbox full
    return Mailbox_post(logging_mailbox, l, BIOS_NO_WAIT);
}


uint8_t logger_pop(struct logger *l)
{
    //var for testing
//    Queue_Handle logging_mailbox;
    uint8_t ret = Mailbox_pend(logging_mailbox, l, BIOS_NO_WAIT);

    // check crc
    return ret;
}

void logging_parse_buffer (char *buffer, log_data_t * decoded)
{
    struct logger l;
    strcpy(l.data, buffer);
    char str_buf[4];
    uint32_t str_buf_sz = sizeof(str_buf);


    cmp_mem_access_ro_init(&l.ctx, &l.cma, l.data, sizeof(l.data));
    cmp_read_str(&l.ctx, str_buf, &str_buf_sz);

    //using the sting, we know how to parse the data
     if (strcmp(str_buf, "gps") == 0){
        decoded->data_type = GPS;
        logging_parse_gps(&l, decoded);
     }
    else if (strcmp(str_buf, "imu") == 0){
        decoded->data_type = IMU;
        logging_parse_imu(&l, decoded);
    }
    else if (strcmp(str_buf, "wea") == 0){
        decoded->data_type = WEATHER;
        logging_parse_weather(&l, decoded);
    }
    
    cmp_read_u32(&l->ctx, &decoded->timestamp);

}

void logging_parse_gps (struct logger * l, log_data_t * decoded)
{

    cmp_read_float(&l->ctx, &decoded->lat);
    cmp_read_float(&l->ctx, &decoded->lon);
    cmp_read_u8(&l->ctx, &decoded->fix_qual);
}
  

void logging_parse_weather (struct logger *l, log_data_t * decoded)
{

    cmp_read_uint(&l->ctx, &decoded->int_press);;
    cmp_read_int(&l->ctx, &decoded->int_temp);
    cmp_read_uint(&l->ctx, &decoded->int_humi);
    cmp_read_uint(&l->ctx, &decoded->ex_press);
    cmp_read_int(&l->ctx, &decoded->ex_temp);
    cmp_read_uint(&l->ctx, &decoded->ex_humi);
}

void logging_parse_imu (struct logger *l, log_data_t * decoded)
{
    cmp_read_u8(&l->ctx, &decoded->imu_calib);
    cmp_read_u16(&l->ctx, &decoded->imu_head);
    cmp_read_u16(&l->ctx, &decoded->imu_roll);
    cmp_read_u16(&l->ctx, &decoded->imu_pitch);
}
