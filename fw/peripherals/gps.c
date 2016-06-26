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

//mavlink includes
#include "../peripherals/comm.h"
#include "../lib/mavlink/common/mavlink.h"


static struct minmea_sentence_gga gps_gga_frame;
static struct minmea_sentence_rmc gps_rmc_frame;
static struct minmea_sentence_gsa gps_gsa_frame; //needed for MAVLINK infos
static struct timespec gps_last_update;

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
	return minmea_tocoord(&gps_rmc_frame.speed);
}

int32_t gps_get_int_speed(){
	return (int32_t)(100*minmea_tocoord(&gps_rmc_frame.speed));
}

float gps_get_course(){
	return minmea_tocoord(&gps_rmc_frame.course);
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


COMM_FRAME* gps_pack_mavlink_raw_int()
{
	// Mavlink heartbeat
	// Define the system type, in this case an airplane
	int32_t lat = (int32_t)(10000000.0 * gps_get_lat()); //Latitude (WGS84), in degrees * 1E7
	int32_t lon = (int32_t)(10000000.0 * gps_get_lon()); //Longitude (WGS84), in degrees * 1E7
	int32_t alt = (int32_t)(1000.0 * gps_get_int_altitude());//Altitude (AMSL, NOT WGS84), in meters * 1000 (positive for up). Note that virtually all GPS modules provide the AMSL altitude in addition to the WGS84 altitude.
	uint16_t cog = (uint16_t)(100.0 * gps_get_course());// Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
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

	/* MAVLINK HEARTBEAT */
	// Initialize the message buffer
	static COMM_FRAME frame;

	// Pack the message
	mavlink_msg_gps_raw_int_pack(mavlink_system.sysid, MAV_COMP_ID_GPS, &(frame.mavlink_message),
		usec, gps_get_fix_type(), lat, lon, alt, gps_get_hdop(), gps_get_vdop(),
		(uint16_t)gps_get_int_speed(), cog, (uint8_t)gps_get_satellites_tracked());
	return &frame;
}


 void gps_task(){

	char *saveptr1 = NULL; 

    cli_init();

	while (1) {

		//initialise GPS device, open UART
		if(!ublox_6_open()){
//			serial_printf(stdout, "%d GPS UART error", 0);
		}

		//get data from device (blocking call)
		char * nmeabuffer = ublox_6_read();
		char * nmeaframes = strtok_r(nmeabuffer, "\n", &saveptr1);

		//parse gps data, this is non-deterministic as data just flys in on the serial line.
		// this then looks for a valid nmea frame

		while (nmeaframes != NULL){
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
			}

			nmeaframes = strtok_r(NULL, "\n", &saveptr1);
		}

		serial_printf(stdout, "dgps_age:%d\n", gps_get_hdop());

		ublox_6_close();
		Task_sleep(1000);

#ifdef MAVLINK_ON_UART0_ENABLED
	comm_set_tx_flag(CHANNEL_APP_UART, MAV_COMP_ID_GPS);
	comm_mavlink_broadcast(gps_pack_mavlink_raw_int());
#endif


	}


}

