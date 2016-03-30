#include "greatest/greatest.h"
#include "../core/log.h"

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#define GPS_ID 0
#define IMU_ID 1
#define WEA_ID 2

int is_eq (log_data_t dec, int type)
{
    int ret = -1;

    if (type == GPS_ID){
        ret = ( 
                (dec.lat == gps_get_lat()) && 
                (dec.lon == gps_get_lon()) && 
                (dec.fix_qual == gps_get_fix_quality()));
    }
    else if (type == IMU_ID){
        ret = ( 
                (dec.imu_calib == imu_get_calib_status())&& 
                (dec.imu_head == imu_get_heading()) && 
                (dec.imu_roll == imu_get_roll()) && 
                (dec.imu_pitch == imu_get_pitch()));
    }
    else if (type == WEA_ID){
        ret =   ((dec.int_press == weather_get_int_press()) && 
                (dec.int_temp == weather_get_int_temp()) && 
                (dec.int_humi == weather_get_int_humid()) && 
                (dec.ex_press == weather_get_ext_press()) && 
                (dec.ex_temp == weather_get_ex_temp()) && 
                (dec.ex_humi == weahter_get_ext_humid()));
    }

    return ret;
}


TEST gps_test(void)
{
    struct logger l;
    log_data_t dec;
    
    printf("%f %f %d \n", gps_get_lat(), gps_get_lon(), gps_get_fix_quality());

    cmp_mem_access_init(&l.ctx, &l.cma, l.data, sizeof(l.data));
    logging_gps_serialize(&l);
    
    int serialized_len = cmp_mem_access_get_pos(&l.cma);
    int i;
    for (i = 0; i < serialized_len; i++) {
        printf("%02x ", (uint8_t)l.data[i]);
    }
    
    printf("\n");

    cmp_mem_access_ro_init(&l.ctx, &l.cma, l.data, sizeof(l.data));
    logging_parse_gps(&l, &dec);
    
    printf("%f %f %d \n", dec.lat, dec.lon, dec.fix_qual);

    ASSERT(is_eq(dec, GPS_ID));
    PASS();
}

TEST imu_test(void)
{
    struct logger l;
    log_data_t dec;

    cmp_mem_access_init(&l.ctx, &l.cma, l.data, sizeof(l.data));
    logging_imu_serialize(&l);

    cmp_mem_access_ro_init(&l.ctx, &l.cma, l.data, sizeof(l.data));
    logging_parse_imu(&l, &dec);

    ASSERT(is_eq(dec, IMU_ID));
    PASS();
}


TEST weather_test(void)
{
    struct logger l;
    log_data_t dec;
    
    cmp_mem_access_init(&l.ctx, &l.cma, l.data, sizeof(l.data));
    logging_weather_serialize(&l);

    cmp_mem_access_ro_init(&l.ctx, &l.cma, l.data, sizeof(l.data));
    logging_parse_weather(&l, &dec);

    ASSERT(is_eq(dec, WEA_ID));
    PASS();
}


SUITE(cmp_test)
{
    RUN_TEST(gps_test);
    RUN_TEST(imu_test);
    RUN_TEST(weather_test);
}

GREATEST_MAIN_DEFS();

int main (int argc, char** argv)
{

    GREATEST_MAIN_BEGIN();
    
    RUN_SUITE(cmp_test);

    GREATEST_MAIN_END();
}
