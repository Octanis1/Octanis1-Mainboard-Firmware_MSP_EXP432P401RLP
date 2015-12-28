/*
 * motors.h
 *
 *  Created on: 13 Oct 2015
 *      Author: raffael
 */

#ifndef FW_PERIPHERALS_HAL_MOTORS_H_
#define FW_PERIPHERALS_HAL_MOTORS_H_

#define N_WHEELS			4
#define WHEEL_RADIUS		0.1 //[m]
#define SPEED_FACTOR		0.00001875 //[rps/pwm value]

//PWM speed scale (percentage) :
#define PWM_SPEED_100	32767
#define PWM_SPEED_80		26214
#define PWM_SPEED_60		19660
#define PWM_SPEED_50		16384
#define PWM_SPEED_40		13107
#define PWM_SPEED_20		6553
#define PWM_SPEED_10		3277
#define PWM_SPEED_0		0



#define N_STRUTS		4


#include <stdint.h>

int motors_init();
int motors_pwm_init();
void motors_pwm_close();

/*
 * arguments are integers scaled to the speed 0 = 0% and 32767 = 100% and the sign of the number
 * determines the direction of travel (negative: backwards, positive: forward).
 */
void motors_wheels_move(int16_t front_left, int16_t front_right, int16_t rear_left, int16_t rear_right);
void motors_struts_move(int8_t front_left, int8_t front_right, int8_t rear_left, int8_t rear_right);

void motors_wheels_stop();
void motors_struts_stop();

void motors_wheels_update_distance();



#endif /* FW_PERIPHERALS_HAL_MOTORS_H_ */
