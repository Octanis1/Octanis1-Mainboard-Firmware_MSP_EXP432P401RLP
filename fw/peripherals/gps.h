#ifndef __GPS_H
#define __GPS_H

#include <stdint.h>

#define NUM_SENTENCES 4

void gps_task();

uint8_t gps_update_new_position(double* lat_, double* lon_);

bool gps_valid();

uint8_t gps_get_fix_quality();
int gps_get_satellites_tracked();

//unscaled lat and lon integers
double gps_get_lat();
double gps_get_lon();

int32_t gps_get_lat_int();// 	Latitude, expressed as degrees * 1E7
int32_t gps_get_lon_int();// 	Longitude, expressed as degrees * 1E7

float gps_get_speed();
int32_t gps_get_int_speed();

float gps_get_altitude();
uint32_t gps_get_int_altitude();
float gps_get_course();

//returns heading calculated as clockwise angle (in centidegrees) with north equal zero.
int32_t gps_get_gps_heading();

// heading angle in degrees (direction of movement)
float gps_get_gps_fheading();

int gps_get_lat_scale();
int gps_get_lon_scale();

int gps_get_validity();
int gps_get_dgps_age();
uint16_t gps_get_hdop();
uint16_t gps_get_vdop();

int gps_get_last_update_time();

#endif
