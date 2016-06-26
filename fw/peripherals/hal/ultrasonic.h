/*
 * ultrasonic.h
 *
 *  Created on: Sep 11, 2015
 *      Author: vagrant
 */

#ifndef FW_PERIPHERALS_HAL_ULTRASONIC_H_
#define FW_PERIPHERALS_HAL_ULTRASONIC_H_

#define N_ULTRASONIC_SAMPLES				4
#define N_ULTRASONIC_ARRAYS				2
#define N_ULTRASONIC_SENSORS_PER_ARRAY	4
#define N_ULTRASONIC_SENSORS 	N_ULTRASONIC_SENSORS_PER_ARRAY * N_ULTRASONIC_ARRAYS

/* Reorder the sensor array from bottom left to bottom right, in a circle */
#define ULTRASONIC_INDEX_0		1
#define ULTRASONIC_INDEX_1		5
#define ULTRASONIC_INDEX_2		3
#define ULTRASONIC_INDEX_3		4
#define ULTRASONIC_INDEX_4		2
#define ULTRASONIC_INDEX_5		6
#define ULTRASONIC_INDEX_6		0
#define ULTRASONIC_INDEX_7		7

//Which ultrasonic are looking forward
#define US_FRONT_1 	2
#define US_FRONT_2 	5
#define US_LEFT		1
#define US_RIGHT	6

#define ULTRASONIC_MAX_SENSOR_VALUE	800

#define CRITICAL_DISTANCE_THRESHOLD_METERS	1
#define SOUND_SPEED	343 // in m/s, at 20C (so change it for antartica)
#define CRITICAL_DISTANCE_THRESHOLD_TIME	((uint16_t) 1000000.0*CRITICAL_DISTANCE_THRESHOLD_METERS/SOUND_SPEED) // in us, perform computation in floating point
#define CRITICAL_DISTANCE_THRESHOLD_TICKS	400

#include "../../../Board.h"

//#include <ti/sysbios/knl/Clock.h>




void ultrasonic_init();

void ultrasonic_set_br(float a, float b, float c, float d, float e, float f, float g, float h);
void ultrasonic_set_bl(float a, float b, float c, float d, float e, float f, float g, float h);

int32_t ultrasonic_get_smallest (int32_t *distance_values, uint8_t size);

/* returns 1 if success */
uint8_t ultrasonic_get_distance(int32_t* distance_values);

uint8_t ultrasonic_check_distance(int32_t dist[], int32_t motor_values[], int32_t motor_scaling_factor);

void ultrasonic_send_pulses(uint8_t index);
void ultrasonic_ISR(uint8_t index);

void ultrasonic_ccr_ISR(uint8_t index, uint16_t timestamp, uint8_t edgetype);

#endif /* FW_PERIPHERALS_HAL_ULTRASONIC_H_ */
