#ifndef __GPS_H
#define __GPS_H

#define NUM_SENTENCES 4

void gps_task();

uint8_t gps_get_fix_quality();
int gps_get_satellites_tracked();

//unscaled lat and lon integers
float gps_get_lat();
float gps_get_lon();

int gps_get_lat_scale();
int gps_get_lon_scale();

int gps_get_validity();
int gps_get_hdop();

int gps_get_last_update_time();

#endif
