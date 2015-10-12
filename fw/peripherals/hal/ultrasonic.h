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

#include "../../../Board.h"

//#include <ti/sysbios/knl/Clock.h>




void ultrasonic_init();

/* returns 1 if success */
bool ultrasonic_get_distance(int32_t* distance_values);


void ultrasonic_send_pulses(uint8_t index);
void ultrasonic_ISR(uint8_t index);

void ultrasonic_ccr_ISR(uint8_t index, uint16_t timestamp, uint8_t edgetype);

#endif /* FW_PERIPHERALS_HAL_ULTRASONIC_H_ */
