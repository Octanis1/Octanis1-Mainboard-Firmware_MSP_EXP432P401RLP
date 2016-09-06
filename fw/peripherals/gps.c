/*
 *  File: gps.c
 *  Description: Model for gps module
 *  Author: Sam
 */


/********************** TODO: this could maybe be merged to the navigation task ****************/

#if defined(NAVIGATION_TEST)

#else
#include "../../Board.h"
#endif

#include "hal/ublox_6.h"
#include "../lib/minmea/minmea.h"
#include "gps.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

//mavlink includes
#include "imu.h"
#include "../peripherals/comm.h"
#include "../core/cli.h"
#include "../lib/mavlink/common/mavlink.h"
#include "hal/time_since_boot.h"

#define POINTS_FOR_HEADING 10
#define MEGA			   1000000

static struct gps {
	float lat_points[POINTS_FOR_HEADING];
	float lon_points[POINTS_FOR_HEADING];
	int32_t gps_heading;
} gps;

static struct minmea_sentence_gga gps_gga_frame;
static struct minmea_sentence_rmc gps_rmc_frame;
static struct minmea_sentence_gsa gps_gsa_frame; //needed for MAVLINK infos
static struct minmea_sentence_vtg gps_vtg_frame;
static struct timespec gps_last_update;

bool gps_valid()
{
	//TODO: wait to get a certain precision
	return gps_get_lat() != 0;
}

uint8_t gps_get_fix_quality(){
	return gps_gga_frame.fix_quality;
}

uint8_t gps_get_fix_type(){
	return gps_gsa_frame.fix_type;
}

int gps_get_satellites_tracked(){
	return gps_gga_frame.satellites_tracked;
}

float gps_get_lat(){
	return minmea_tocoord(&gps_rmc_frame.latitude);
}

float gps_get_lon(){
	return minmea_tocoord(&gps_rmc_frame.longitude);
}

int32_t gps_get_lat_int(){// 	Latitude, expressed as degrees * 1E7
	return 0;//minmea_rescale(&gps_rmc_frame.latitude, 100000); //TODO: MAKE CORRECT CONVERSION (NMEA message is in minutes, etc)
}

int32_t gps_get_lon_int(){ //  Longitude, expressed as degrees * 1E7
	return 0;//minmea_rescale(&gps_rmc_frame.longitude, 100000);
}

float gps_get_speed(){
	return minmea_tofloat(&gps_rmc_frame.speed);
}

int32_t gps_get_int_speed(){
	return (int32_t)(100*minmea_tofloat(&gps_rmc_frame.speed));
}

float gps_get_course(){
	return minmea_tofloat(&gps_rmc_frame.course);
}

float gps_get_cog(){
	return minmea_tofloat(&gps_vtg_frame.true_track_degrees);
}

float gps_get_altitude(){
	return minmea_tofloat(&gps_gga_frame.altitude);
}

uint32_t gps_get_int_altitude(){
	return (int32_t)minmea_tofloat(&gps_gga_frame.altitude);
}

int gps_get_lat_scale(){
	return gps_gga_frame.latitude.scale;
}

int gps_get_lon_scale(){
	return gps_gga_frame.longitude.scale;
}

int gps_get_validity(){
	return gps_rmc_frame.valid;
}

uint16_t gps_get_hdop(){
	return minmea_tofloat(&gps_gsa_frame.hdop)*100;
}
uint16_t gps_get_vdop(){
	return minmea_tofloat(&gps_gsa_frame.vdop)*100;
}

int gps_get_dgps_age(){
	return gps_gga_frame.dgps_age;
}

int gps_get_last_update_time(){
	minmea_gettime(&gps_last_update, &gps_rmc_frame.date, &gps_rmc_frame.time);
	return gps_last_update.tv_sec;
}

uint64_t gps_get_last_update_usec()
{
	return gps_rmc_frame.time.microseconds;
}

uint8_t gps_update_new_position(float* lat_, float* lon_)
{
	if(gps_get_validity())
	{
		(*lat_)=gps_get_lat();
		(*lon_)=gps_get_lon();
		//return 1 if the gps position has changed significatly.
		return 1;
	}

	//else: no update performed.
	return 0;
}

void gps_run_gps_heading()
{
	uint8_t i;
	for (i = 0; i < (POINTS_FOR_HEADING - 1); i++) {
		gps.lat_points[i+1] = gps.lat_points[i];
		gps.lon_points[i+1] = gps.lon_points[i];
	}
	gps.lat_points[0] = gps_get_lat();
	gps.lon_points[0] = gps_get_lon();
}

//heading calculated as clockwise angle (in mrad) with north equal zero.
int32_t gps_get_gps_heading()
{
	int8_t i, j;
	int32_t gps_heading;
	float old_average_lon = 0;
	float old_average_lat = 0;
	float new_average_lon = 0;
	float new_average_lat = 0;
	float delta_lon=0;
	float delta_lat = 0;
	float alpha = 0;


	for (i = 0; i < (POINTS_FOR_HEADING/2); i++) {
		new_average_lon += gps.lon_points[i];
		new_average_lat += gps.lat_points[i];
	}
	for (j = (POINTS_FOR_HEADING/2); j < POINTS_FOR_HEADING; j++) {
		old_average_lon += gps.lon_points[j];
		old_average_lat += gps.lat_points[j];
	}
	delta_lon = (new_average_lon - old_average_lon) / (POINTS_FOR_HEADING/2);
	delta_lat = (new_average_lat - old_average_lat) / (POINTS_FOR_HEADING/2);
	alpha = atan2f(delta_lon, delta_lat)*57.2957795+90; // *180/pi

	if(alpha>360)
		alpha = alpha - 360;

	gps_heading = (int32_t)(1000 * alpha);
	return gps_heading;
}

COMM_FRAME* gps_pack_mavlink_global_position_int()
{
	// Mavlink heartbeat
	// Define the system type, in this case an airplane
	int32_t lat = (int32_t)(10000000.0 * gps_get_lat()); //Latitude (WGS84), in degrees * 1E7
	int32_t lon = (int32_t)(10000000.0 * gps_get_lon()); //Longitude (WGS84), in degrees * 1E7
	int32_t alt = (int32_t)(1000.0 * gps_get_int_altitude());//Altitude (AMSL, NOT WGS84), in meters * 1000 (positive for up). Note that virtually all GPS modules provide the AMSL altitude in addition to the WGS84 altitude.
	int32_t relative_alt = 0;
	int16_t vx = 0;
	int16_t vy = 0;
	int16_t vz = 0;
	uint16_t hdg = gps_get_gps_heading();
	uint32_t msec = ms_since_boot();

	// Initialize the message buffer
	static COMM_FRAME frame;

	// Pack the message
	mavlink_msg_global_position_int_pack(mavlink_system.sysid, MAV_COMP_ID_GPS, &(frame.mavlink_message),
				msec, lat, lon, alt, relative_alt,  vx, vy, vz, hdg);
	return &frame;
}


COMM_FRAME* gps_pack_mavlink_raw_int()
{
	// Mavlink heartbeat
	// Define the system type, in this case an airplane
	int32_t lat = (int32_t)(10000000.0 * gps_get_lat()); //Latitude (WGS84), in degrees * 1E7
	int32_t lon = (int32_t)(10000000.0 * gps_get_lon()); //Longitude (WGS84), in degrees * 1E7
	int32_t alt = (int32_t)(1000.0 * gps_get_int_altitude());//Altitude (AMSL, NOT WGS84), in meters * 1000 (positive for up). Note that virtually all GPS modules provide the AMSL altitude in addition to the WGS84 altitude.
	uint16_t cog = (uint16_t)(100.0 * gps_get_cog());// Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
	uint64_t usec = gps_get_last_update_usec();
	if(usec == 0)
	{ //no time information in usec is available
		usec = (uint64_t)gps_get_last_update_time();
		if(usec == 0)
			//no time information from GPS is available --> use system time (since boot)
			usec = us_since_boot();
		else
			usec = 1000000 * usec;
	}

	// Initialize the message buffer
	static COMM_FRAME frame;

	// Pack the message
	mavlink_msg_gps_raw_int_pack(mavlink_system.sysid, MAV_COMP_ID_GPS, &(frame.mavlink_message),
		usec, gps_get_fix_type(), lat, lon, alt, gps_get_hdop(), gps_get_vdop(), //TODO: change back 2nd argument to gps_get_fix_type() or make it conform to mavlink standard
		(uint16_t)gps_get_int_speed(), cog, (uint8_t)gps_get_satellites_tracked());
	return &frame;
}

void gps_task(){
    cli_init();

    ublox_6_open();

	while (1) {


		//initialise GPS device, open UART


		//get data from device (blocking call)
		static char * nmeaframes;

		//parse gps data, this is non-deterministic as data just flys in on the serial line.
		// this then looks for a valid nmea frame

		while ((nmeaframes = ublox_6_read())){
			int minmea_sentence = minmea_sentence_id(nmeaframes, false);

			switch (minmea_sentence) {
				case MINMEA_SENTENCE_GGA:
				{
					minmea_parse_gga(&gps_gga_frame, nmeaframes);
					break;
				}
				case MINMEA_SENTENCE_RMC:
				{
					if(minmea_parse_rmc(&gps_rmc_frame, nmeaframes)){
						//update system time when valid RMC frame arrives
						Seconds_set(gps_get_last_update_time());
					}
					break;
				}
				case MINMEA_SENTENCE_GSA:
				{
					minmea_parse_gsa(&gps_gsa_frame, nmeaframes);
				}
				case MINMEA_SENTENCE_VTG:
					minmea_parse_vtg(&gps_vtg_frame, nmeaframes);
			}

			if(minmea_sentence == MINMEA_SENTENCE_RMC)
				break;

		}

		gps_run_gps_heading();


		if((gps_rmc_frame.longitude.value != 0) && (gps_rmc_frame.latitude.value != 0) && (gps_rmc_frame.longitude.scale != 0) && (gps_rmc_frame.latitude.scale != 0))
		{
			comm_mavlink_broadcast(gps_pack_mavlink_raw_int());

			Task_sleep(10);

			comm_mavlink_broadcast(gps_pack_mavlink_global_position_int());
		}

		Task_sleep(10);

	}
}

