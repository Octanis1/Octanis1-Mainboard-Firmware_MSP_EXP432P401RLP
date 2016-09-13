///*
// * vision.c
// *
// *  Created on: Sep 12, 2015
// *      Author: vagrant
// */
//
//#include "vision.h"
//
//#include "hal/ultrasonic.h"
//#include "../../Board.h"
//#include "../core/eps.h"
//
//
//void vision_task(){
//	cli_init();
//
////	ultrasonic_init();
//	eps_init();
//
//	int32_t distance_values[N_ULTRASONIC_SENSORS_PER_ARRAY*N_ULTRASONIC_ARRAYS];
//	int8_t directions_result[N_ULTRASONIC_SENSORS_PER_ARRAY*N_ULTRASONIC_ARRAYS] = {(int8_t) 1}; //all ok
//
//
//	while(1){
////		eps_switch_module(M3V3_1_ON);
////		if(ultrasonic_get_distance(distance_values))
////		{
////		//successfully read the sensor values
//////			serial_printf(cli_stdout, 'us check %d \n', (int)CRITICAL_DISTANCE_THRESHOLD_TIMESTAMP);
////			ultrasonic_check_distance(distance_values, directions_result);
////		}
////		else
////		{
////		//not all or none of the sensors returned a pulse
////		}
//		Task_sleep(5000);
////		eps_switch_module(M3V3_1_OFF);
//
//	}
//
//}
