/*
 * ultrasonic.h
 *
 *  Created on: Sep 11, 2015
 *      Author: vagrant
 */

#ifndef FW_PERIPHERALS_HAL_ULTRASONIC_H_
#define FW_PERIPHERALS_HAL_ULTRASONIC_H_

#include "../../../Board.h"

//#include <ti/sysbios/knl/Clock.h>




void ultrasonic_init();

void ultrasonic_send_pulse(uint8_t index);
void ultrasonic_ISR(uint8_t index);

void ultrasonic_ccr_ISR(uint8_t index, uint16_t timestamp, uint8_t edgetype);

#endif /* FW_PERIPHERALS_HAL_ULTRASONIC_H_ */
