#ifndef __GPS_H
#define __GPS_H

#include <stdint.h>

#define NUM_SENTENCES 4

void gps_task();

uint8_t gps_update_new_position(float* lat_, float* lon_);

bool gps_valid();

uint8_t gps_get_fix_quality();
int gps_get_satellites_tracked();

//unscaled lat and lon integers
float gps_get_lat(); //in degrees
float gps_get_lon(); //in degrees

float gps_get_speed();
int32_t gps_get_int_speed();

float gps_get_altitude();
uint32_t gps_get_int_altitude();
float gps_get_course();

float gps_get_cog(); //in degrees

int gps_get_lat_scale();
int gps_get_lon_scale();

int gps_get_validity();
int gps_get_dgps_age();
uint16_t gps_get_hdop();
uint16_t gps_get_vdop();

int gps_get_last_update_time();

void gps_run_gps(uint8_t position_i);
void gps_calculate_position();
void gps_reset_gps();
int32_t gps_get_latitude();
int32_t gps_get_longitude();

void gps_receive_lat_rover(int32_t lat_rover);
void gps_receive_lon_rover(int32_t lon_rover);

void gps_initialize();

#endif
