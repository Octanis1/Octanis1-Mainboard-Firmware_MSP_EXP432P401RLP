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
#define ESP_LAT 46.520216
#define ESP_LON 6.565783
#define HACK_LAT 46.532483
#define HACK_LON 6.590315
#define EL_LAT 46.519832
#define EL_LON 6.564925
#define ROLEX_LAT 46.518996
#define ROLEX_LON 6.568278
#define PLASMA_LAT 46.517241
#define PLASMA_LON 6.565021

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

	float toPlasma=0;
	float toEL=0;
	float toRolex=0;
	float toHackuarium=0;
	float toEsplanade=0;

	while(1){

		GPIO_write(Board_LED_RED, Board_LED_OFF);
		struct rover_status pos_var;

		//Commented for debug reason because oterwise we never go into the else
		//We check if the gps is already working
		if (gps_get_validity() == 0){
			//error
		}
		else{
			pos_var.lat_current = gps_get_lat();
			pos_var.lon_current = gps_get_lon();

			toPlasma= nav_get_distance(pos_var.lat_current, pos_var.lon_current, PLASMA_LAT, PLASMA_LON);
			toEL= nav_get_distance(pos_var.lat_current, pos_var.lon_current, EL_LAT, EL_LON);
			toRolex= nav_get_distance(pos_var.lat_current, pos_var.lon_current, ROLEX_LAT, ROLEX_LON);
			toHackuarium= nav_get_distance(pos_var.lat_current, pos_var.lon_current, HACK_LAT, HACK_LON);
			toEsplanade= nav_get_distance(pos_var.lat_current, pos_var.lon_current, ESP_LAT, ESP_LON);

			//We check if we are at the location
			if(toEsplanade <= 5){
				System_printf("Esplanade reached\n");
				    /* SysMin will only print to the console when you call flush or exit */
				System_flush();
			}
			else if (toHackuarium <= 5){
				System_printf("Hackuarium reached\n");
								    /* SysMin will only print to the console when you call flush or exit */
				System_flush();
			}
			else if (toEL <= 5){
				System_printf("EL reached\n");
								    /* SysMin will only print to the console when you call flush or exit */
				System_flush();
			}
			else if (toRolex <= 5){
				System_printf("RLC reached\n");
								    /* SysMin will only print to the console when you call flush or exit */
				System_flush();
			}
			else if (toPlasma <= 5){
				System_printf("Plasma center reached\n");
								    /* SysMin will only print to the console when you call flush or exit */
				System_flush();
			}
			else{
				System_printf("To EL = %f\n To Esplanade = %f\n To Rolex = %f\n To Plasma = %f\n To Hackuarium = %f\n", toEL, toEsplanade, toRolex, toPlasma, toHackuarium);
				    /* SysMin will only print to the console when you call flush or exit */
				System_flush();
			}

			pos_var.angle = nav_get_angle(pos_var.lat_current, pos_var.lon_current, HACK_LAT, HACK_LON);

			//Tell the motor to move accordingly
		}


		Task_sleep(1500);
	}
}

