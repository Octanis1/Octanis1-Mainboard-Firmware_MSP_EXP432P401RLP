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
	int32_t distance_values[N_ULTRASONIC_SENSORS_PER_ARRAY*N_ULTRASONIC_ARRAYS];

	while(1){

		if(ultrasonic_get_distance(distance_values))
		{
		//successfully read the sensor values
		}
		else
		{
		//not all or none of the sensors returned a pulse
		}
		Task_sleep(5000);
	}

}
