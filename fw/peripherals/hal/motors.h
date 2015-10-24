/*
 * motors.h
 *
 *  Created on: 13 Oct 2015
 *      Author: raffael
 */

#ifndef FW_PERIPHERALS_HAL_MOTORS_H_
#define FW_PERIPHERALS_HAL_MOTORS_H_

#include <stdint.h>


int motors_pwm_init();
void motors_pwm_close();


void motors_wheels_move(int16_t front_left, int16_t front_right, int16_t rear_left, int16_t rear_right);
void motors_struts_move(int8_t front_left, int8_t front_right, int8_t rear_left, int8_t rear_right);

void motors_wheels_stop();
void motors_struts_stop();


#endif /* FW_PERIPHERALS_HAL_MOTORS_H_ */
