/*
 * motors.h
 *
 *  Created on: 13 Oct 2015
 *      Author: raffael
 */

#ifndef FW_PERIPHERALS_HAL_MOTORS_H_
#define FW_PERIPHERALS_HAL_MOTORS_H_

/* Definitions for wheels */
#define N_WHEELS			4
#define WHEEL_RADIUS		100 		//[mm]
#define SPEED_FACTOR		0.00001875 	//[rps/pwm value]

//PWM speed scale (time in us) :
#define PWM_PERIOD			40
#define PWM_SPEED_100		40
#define PWM_SPEED_80		32
#define PWM_SPEED_70		28
#define PWM_SPEED_60		24
#define PWM_SPEED_50		20
#define PWM_SPEED_40		16
#define PWM_SPEED_20		8
#define PWM_SPEED_10		4
#define PWM_SPEED_0			0

#define PID_SCALING_FACTOR	PWM_SPEED_100/5/45 //turn with maximum wheel speed difference (100%/80%) for 45° bearing error

#define PWM_MINIMUM_SPEED	26 //this is the minimum speed we want so the motors are still visibly moving.

#define PH_FORWARD		0
#define PH_REVERSE		1

#define M_PI 3.14159265358979323846

/* Definitions for struts */

#define N_STRUTS			4

#include <stdint.h>

//Definitions for GPS / odometry cycle
#define MAX_RECENT_VALUES		 150
#define VALUES_AFTER_GPS_RESET   50

int motors_init();
int motors_pwm_init();
void motors_pwm_close();

/*
 * arguments are integers scaled to the speed 0 = 0% and 65535 = 100% and the sign of the number
 * determines the direction of travel (negative: backwards, pos0itive: forward).
 */
void motors_wheels_move(int32_t front_left, int32_t front_right, int32_t rear_left, int32_t rear_right);
void motors_struts_move(int8_t front_left, int8_t front_right, int8_t rear_left, int8_t rear_right);

void motors_wheels_stop();
void motors_struts_stop();

/*Take care of odometry.
 */
uint16_t motors_get_sensor_values(int i);

void motors_struts_get_position();

#endif /* FW_PERIPHERALS_HAL_MOTORS_H_ */
