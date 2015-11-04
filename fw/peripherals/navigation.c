/*
 * navigation.c
 *
 *  Created on: Oct 24, 2015
 *      Author: Eloi Benvenuti
 *
 *  File description: Module for computing the path from position to target and sending command to motors.
 *  distances are in meters and angles in radian.
 */

#include "../../Board.h"
#include "gps.h"
#include "navigation.h"


#define EARTH_RADIUS 6356752

//Target for testing, just outside of hackuarium
//#define TARGET_LAT 46.532476
//#define TARGET_LON 6.590315

//Other, hopefully closer
#define TARGET_LAT 76.31559
#define TARGET_LON 6.35276

struct rover_status{
	float lat_current;
	float lon_current;
	float distance;
	float angle;
};

float nav_get_distance(float lat_current, float lon_current, float lat_target, float lon_target){

	/*About this code: here we use the haversin formula
	 * As I undersood, we may run into problems if our robot goes past the 180/-180 degree line
	 * As I get it, we have a 0.5% error with this formula, even with us being at the pole
	 * However it would be nice to double check that
	 * 0.5% error means that, on a 5m distance, we are 25cm imprecise.
	 * Since the gps is probably 5m off, it's not a big deal.
	 */
	float distance = 0;

	distance = 2*EARTH_RADIUS*asinf(sqrtf(powf(sinf((lat_target-lat_current)/2),2) + cosf(lat_current)*cosf(lat_target)*powf(sinf((lon_target-lon_current)/2),2)));

	return distance;
}

float nav_get_angle(float lat_current, float lon_current, float lat_target, float lon_target){
	float angle = 0;

	return angle;
}

void navigation_task(){

	while(1){

		GPIO_write(Board_LED_RED, Board_LED_OFF);
		struct rover_status pos_var;

		//Commented for debug reason because oterwise we never go into the else
		//We check if the gps is already working
		if (gps_get_validity() == 0){
		//if(0){
		}
		else{
			pos_var.lat_current = gps_get_lat();
			pos_var.lon_current = gps_get_lon();



			pos_var.distance = nav_get_distance(pos_var.lat_current, pos_var.lon_current, TARGET_LAT, TARGET_LON);

			//We check if we are at the location
			if(pos_var.distance <= 5){
				GPIO_write(Board_LED_RED, Board_LED_ON);
			}


			pos_var.angle = nav_get_angle(pos_var.lat_current, pos_var.lon_current, TARGET_LAT, TARGET_LON);

			//Tell the motor to move accordingly
		}


		Task_sleep(1500);
	}
}

