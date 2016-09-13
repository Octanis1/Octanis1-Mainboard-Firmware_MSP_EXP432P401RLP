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
#include "navigation.h"
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

#define POINTS_FOR_HEADING 10 //must be even number!!
#define MEGA			   1000000
#define PI 3.14159265

// store the last 10 numbers
int32_t lat_points[POINTS_FOR_HEADING];
int32_t lon_points[POINTS_FOR_HEADING];

static struct minmea_sentence_gga gps_gga_frame;
static struct minmea_sentence_rmc gps_rmc_frame;
static struct minmea_sentence_gsa gps_gsa_frame; //needed for MAVLINK infos
static struct minmea_sentence_vtg gps_vtg_frame;
static struct timespec gps_last_update;

bool gps_valid()
{
	//TODO: wait to get a certain precision
	return gps_get_lat_int() != 0;
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

double gps_get_lat(){
	return minmea_tocoord(&gps_rmc_frame.latitude);
}

double gps_get_lon(){
	return minmea_tocoord(&gps_rmc_frame.longitude);
}

int32_t gps_get_lat_int(){// 	Latitude, expressed as degrees * 1E7
	return minmea_tocoord_int(&gps_rmc_frame.latitude); //TODO: MAKE CORRECT CONVERSION (NMEA message is in minutes, etc)
}

int32_t gps_get_lon_int(){ //  Longitude, expressed as degrees * 1E7
	return minmea_tocoord_int(&gps_rmc_frame.longitude);
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

uint8_t gps_update_new_position(double* lat_, double* lon_)
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

static int recalc_heading = 1;
void gps_run_gps_heading()
{
	if(navigation_rover_moving()) //ensure the "heading angle" doesnt get changed when the rover stands still.
	{
		uint8_t i;
		for (i = POINTS_FOR_HEADING-1; i > 0 ; i--) {
			lat_points[i] = lat_points[i-1];
			lon_points[i] = lon_points[i-1];
		}
		lat_points[0] = gps_get_lat_int();
		lon_points[0] = gps_get_lon_int();

		recalc_heading = 1;
	}
}

//heading calculated as clockwise angle (in centidegrees) with north equal zero and +/-180degrees range.
int32_t gps_get_gps_heading()
{
	int8_t i, j;
	static int32_t gps_heading = 0;

	if(recalc_heading)
	{
		int64_t old_average_lon = 0;
		int64_t old_average_lat = 0;
		int64_t new_average_lon = 0;
		int64_t new_average_lat = 0;
		double new_average_lon_d=0;
		double new_average_lat_d = 0;
		double old_average_lon_d=0;
		double old_average_lat_d = 0;
		double delta_lon=0;
		double delta_lat = 0;
		double alpha = 0;

		for (i = 0; i < (POINTS_FOR_HEADING/2); i++) {
			new_average_lon += lon_points[i];
			new_average_lat += lat_points[i];
		}
		for (j = (POINTS_FOR_HEADING/2); j < POINTS_FOR_HEADING; j++) {
			old_average_lon += lon_points[j];
			old_average_lat += lat_points[j];
		}

		new_average_lon_d = (double)new_average_lon / (double)(10000000*POINTS_FOR_HEADING/2);
		new_average_lat_d = (double)new_average_lat / (double)(10000000*POINTS_FOR_HEADING/2);
		old_average_lon_d = (double)old_average_lon / (double)(10000000*POINTS_FOR_HEADING/2);
		old_average_lat_d = (double)old_average_lat / (double)(10000000*POINTS_FOR_HEADING/2);


		delta_lon = (new_average_lon_d - old_average_lon_d);
		delta_lat = (new_average_lat_d - old_average_lat_d);

		if(delta_lon == 0 && delta_lat == 0)
		{
			return gps_heading;
		}

		static double d_x = 0;
		static double d_y = 0;

		// perform moving average with 80% weight on history
		d_x = cos(new_average_lat_d * PI / 180.0 ) * sin(delta_lon* PI / 180.0 );
		d_y = (cos(old_average_lat_d*PI/180.0)*sin(new_average_lat_d*PI/180.0)-sin(old_average_lat_d*PI/180.0)*cos(new_average_lat_d*PI/180.0)*cos(delta_lat*PI/180.0));

		alpha = atan2(d_x, d_y)*57.2957795 ; // *180/pi

		gps_heading = (int32_t)(100 * alpha);

		recalc_heading = 0;
	}

	return gps_heading;
}

//heading calculated as clockwise angle
float gps_get_gps_fheading() //from -180 to 180 degrees
{
	return (float)gps_get_gps_heading() / 100.0;
}

COMM_FRAME* gps_pack_mavlink_global_position_int()
{
	// Mavlink heartbeat
	// Define the system type, in this case an airplane
	int32_t lat = gps_get_lat_int();//(int32_t)(10000000.0 * gps_get_lat()); //Latitude (WGS84), in degrees * 1E7
	int32_t lon = gps_get_lon_int();//(int32_t)(10000000.0 * gps_get_lon()); //Longitude (WGS84), in degrees * 1E7
	int32_t alt = (int32_t)(1000.0 * gps_get_int_altitude());//Altitude (AMSL, NOT WGS84), in meters * 1000 (positive for up). Note that virtually all GPS modules provide the AMSL altitude in addition to the WGS84 altitude.
	int32_t relative_alt = 0;
	int16_t vx = 0;
	int16_t vy = 0;
	int16_t vz = 0;
	int32_t hdg = gps_get_gps_heading();
	if(hdg < 0)
		hdg = hdg + 36000;
	uint32_t msec = ms_since_boot();

	// Initialize the message buffer
	static COMM_FRAME frame;

	// Pack the message
	mavlink_msg_global_position_int_pack(mavlink_system.sysid, MAV_COMP_ID_GPS, &(frame.mavlink_message),
				msec, lat, lon, alt, relative_alt,  vx, vy, vz, (uint16_t)hdg);
	return &frame;
}


COMM_FRAME* gps_pack_mavlink_raw_int()
{
	// Mavlink heartbeat
	// Define the system type, in this case an airplane
	int32_t lat = gps_get_lat_int(); //Latitude (WGS84), in degrees * 1E7
	int32_t lon = gps_get_lon_int(); //Longitude (WGS84), in degrees * 1E7
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

