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
#include "hal/motors.h"
#include "navigation.h"
#include "hal/time_since_boot.h"

//mavlink includes
#include "imu.h"
#include "../peripherals/comm.h"
#include "../core/cli.h"
#include "../lib/mavlink/common/mavlink.h"

#define E_SEVEN 10000000

static struct minmea_sentence_gga gps_gga_frame;
static struct minmea_sentence_rmc gps_rmc_frame;
static struct minmea_sentence_gsa gps_gsa_frame; //needed for MAVLINK infos
static struct minmea_sentence_vtg gps_vtg_frame;
static struct timespec gps_last_update;

static struct gps{
	float first_third_latitude;
	float second_third_latitude;
	float third_third_latitude;
	float first_third_longitude;
	float second_third_longitude;
	float third_third_longitude;
	int32_t position_latitude;		//in degrees e7
	int32_t position_longitude;		//in degrees e7
	int32_t lat_rover;				//not only from gps
	int32_t lon_rover;				//not only form gps
}gps;

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
	return gps_gsa_frame.hdop.value;
}
uint16_t gps_get_vdop(){
	return gps_gsa_frame.vdop.value;
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

void gps_run_gps(uint8_t position_i)
{
	if (position_i < VALUES_AFTER_GPS_RESET){
		gps.first_third_latitude += gps_get_lat();
		gps.first_third_longitude += gps_get_lon();
	}
	else if (position_i < (MAX_RECENT_VALUES - VALUES_AFTER_GPS_RESET)){
		gps.second_third_latitude += gps_get_lat();
		gps.second_third_longitude += gps_get_lon();
	}
	else{
		gps.third_third_latitude += gps_get_lat();
		gps.third_third_longitude += gps_get_lon();
	}
}

void gps_calculate_position()
{
	gps.position_longitude = (gps.first_third_longitude + gps.second_third_longitude) * E_SEVEN / (MAX_RECENT_VALUES - VALUES_AFTER_GPS_RESET); //in degrees e7
	gps.position_latitude = (gps.first_third_latitude + gps.second_third_latitude) * E_SEVEN / (MAX_RECENT_VALUES - VALUES_AFTER_GPS_RESET);	//in degrees e7
}

void gps_reset_gps()
{
	gps.first_third_latitude = gps.third_third_latitude;
	gps.first_third_longitude = gps.third_third_longitude;
	gps.second_third_latitude = 0;
	gps.second_third_longitude = 0;
	gps.third_third_latitude = 0;
	gps.third_third_longitude = 0;
}
void gps_shift_gps()
{
	gps.first_third_latitude = gps.second_third_latitude;
	gps.first_third_longitude = gps.second_third_longitude;
	gps.second_third_latitude = gps.third_third_latitude;
	gps.second_third_longitude = gps.third_third_longitude;
	gps.third_third_latitude = 0;
	gps.third_third_longitude = 0;
}

//returns latitude as measured by gps in degrees e7
int32_t gps_get_latitude()
{
	return gps.position_latitude;
}

//returns longitude as measured by gps in degrees e7
int32_t gps_get_longitude()
{
	return gps.position_longitude;
}

void gps_receive_lat_rover(int32_t lat_rover)
{
	gps.lat_rover = lat_rover;
}

void gps_receive_lon_rover(int32_t lon_rover)
{
	gps.lon_rover = lon_rover;
}

COMM_FRAME* gps_pack_mavlink_global_position_int()
{
	uint8_t position_i;
	int32_t heading;
	int32_t lon, lat;

	position_i = navigation_get_position_i();
	heading = navigation_get_heading_rover();
	heading = heading / 100; //heading in tenths of degrees

	if(navigation_send_signal()) {
		lon = (int32_t)(gps.lon_rover);
		lat = (int32_t)(gps.lat_rover);
	}
	else {
		lon = (int32_t)(10000000.0 * gps_get_lon());
		lat = (int32_t)(10000000.0 * gps_get_lat());
	}

	int32_t alt = (int32_t)(position_i);

	int32_t relative_alt = navigation_get_angle();
	int16_t vx = navigation_get_radius();

	int16_t vy = navigation_get_vmot2rps_factor();
	int16_t vz = navigation_get_turning_parameter();

	uint16_t hdg = (uint16_t)(heading);

	uint32_t msec = ms_since_boot(); //1000 * (uint32_t)Seconds_get();

	static COMM_FRAME frame;

	mavlink_msg_global_position_int_pack(mavlink_system.sysid, MAV_COMP_ID_GPS, &(frame.mavlink_message), msec, lat, lon,
			alt, relative_alt, vx, vy, vz, hdg);

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
			usec = 1000000 * (uint64_t)Seconds_get();
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

		if((gps_rmc_frame.longitude.value != 0) && (gps_rmc_frame.latitude.value != 0) && (gps_rmc_frame.longitude.scale != 0) && (gps_rmc_frame.latitude.scale != 0))
		{
	#ifdef MAVLINK_ON_LORA_ENABLED
			comm_set_tx_flag(CHANNEL_LORA, MAV_COMP_ID_GPS);
	#endif
/*
	#ifdef MAVLINK_ON_UART0_ENABLED
			comm_set_tx_flag(CHANNEL_APP_UART, MAV_COMP_ID_GPS);
	#endif
			comm_mavlink_broadcast(gps_pack_mavlink_raw_int());
*/
	#ifdef MAVLINK_ON_UART0_ENABLED
			comm_set_tx_flag(CHANNEL_APP_UART, MAV_COMP_ID_GPS);
	#endif
			comm_mavlink_broadcast(gps_pack_mavlink_global_position_int());
		}
		Task_sleep(10);
	}
}
