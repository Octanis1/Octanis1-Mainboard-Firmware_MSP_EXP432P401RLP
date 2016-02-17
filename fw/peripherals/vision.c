/*
 * vision.c
 *
 *  Created on: Sep 12, 2015
 *      Author: vagrant
 */

#include "vision.h"

#include "hal/ultrasonic.h"
#include "../../Board.h"


void vision_task(){
	ultrasonic_init();
	int32_t distance_values[N_ULTRASONIC_SENSORS];
	int8_t directions_result[N_ULTRASONIC_SENSORS] = {(int8_t) 1}; //all ok


	while(1){

		if(ultrasonic_get_distance(distance_values))
		{
		//successfully read the sensor values
//			cli_printf('us check %d \n', (int)CRITICAL_DISTANCE_THRESHOLD_TIMESTAMP);
			ultrasonic_check_distance(distance_values, directions_result);
		}
		else
		{
		//not all or none of the sensors returned a pulse
		}
		Task_sleep(5000);
	}

}
