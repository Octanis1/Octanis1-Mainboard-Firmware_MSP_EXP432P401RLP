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

	while(1){

		ultrasonic_send_pulse(0);
		Task_sleep(5000);
	}

}
