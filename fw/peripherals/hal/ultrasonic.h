/*
 * ultrasonic.h
 *
 *  Created on: Sep 11, 2015
 *      Author: vagrant
 */

#ifndef FW_PERIPHERALS_HAL_ULTRASONIC_H_
#define FW_PERIPHERALS_HAL_ULTRASONIC_H_

#define N_ULTRASONIC_ARRAYS				2
#define N_ULTRASONIC_SENSORS_PER_ARRAY	4

#define CRITICAL_DISTANCE_THRESHOLD_METERS	1
#define SOUND_SPEED	343 // in m/s, at 20C (so change it for antartica)
#define CRITICAL_DISTANCE_THRESHOLD_TIMESTAMP	((uint16_t) 1000000.0*CRITICAL_DISTANCE_THRESHOLD_METERS/SOUND_SPEED) // in us, perform computation in floating point

#include "../../../Board.h"

//#include <ti/sysbios/knl/Clock.h>




void ultrasonic_init();

/* returns 1 if success */
bool ultrasonic_get_distance(int32_t* distance_values);

void ultrasonic_check_distance(int32_t distance_values[], int8_t directions_array[]);


void ultrasonic_send_pulses(uint8_t index);
void ultrasonic_ISR(uint8_t index);

void ultrasonic_ccr_ISR(uint8_t index, uint16_t timestamp, uint8_t edgetype);

#endif /* FW_PERIPHERALS_HAL_ULTRASONIC_H_ */
