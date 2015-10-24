/*
 *  File: gps.c
 *  Description: Model for gps module
 *  Author: Sam
 */
#include "../../Board.h"
#include "hal/ublox_6.h"
#include "../lib/minmea/minmea.h"
#include "gps.h"


static struct minmea_sentence_gga gps_gga_frame;
static struct minmea_sentence_rmc gps_rmc_frame;
static struct timespec gps_last_update;

int gps_get_fix_quality(){
	return gps_gga_frame.fix_quality;
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

int gps_get_lat_scale(){
	return gps_gga_frame.latitude.scale;
}

int gps_get_lon_scale(){
	return gps_gga_frame.longitude.scale;
}

int gps_get_validity(){
	return gps_rmc_frame.valid;
}

int gps_get_hdop(){
	return gps_gga_frame.hdop.value;
}


int gps_get_last_update_time(){
	minmea_gettime(&gps_last_update, &gps_rmc_frame.date, &gps_rmc_frame.time);
	return gps_last_update.tv_sec;
}

void gps_task(){

	while (1) {

		//initialise GPS device, open UART
		if(!ublox_6_open()){
			cli_printf("%d GPS UART error", 0);
		}

		//get data from device (blocking call)
		char * nmeabuffer = ublox_6_read();
		char * nmeaframes = strtok(nmeabuffer, "\n");

		//parse gps data, this is non-deterministic as data just flys in on the serial line.
		// this then looks for a valid nmea frame

		while (nmeaframes != NULL){
			int minmea_sentence = minmea_sentence_id(nmeaframes, false);

			switch (minmea_sentence) {
				case MINMEA_SENTENCE_GGA: {
					minmea_parse_gga(&gps_gga_frame, nmeaframes);
				}break;

				case MINMEA_SENTENCE_RMC: {
					if(minmea_parse_rmc(&gps_rmc_frame, nmeaframes)){
						//update system time when valid RMC frame arrives
						Seconds_set(gps_get_last_update_time());
					}

				}break;
			}

			nmeaframes = strtok(NULL, "\n");
		}

		ublox_6_close();
		Task_sleep(6000);
	}


}
