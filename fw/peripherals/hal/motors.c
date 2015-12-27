/*
 * motors.c
 *
 *  Created on: 13 Oct 2015
 *      Author: raffael
 */

#include "motors.h"
#include "../../../Board.h"
#include "adc.h"

PWM_Handle pwm5_handle, pwm6_handle, pwm7_handle, pwm8_handle;
PWM_Params pwm5_params, pwm6_params, pwm7_params, pwm8_params;


int motors_init()
{
	motors_pwm_init();
	adc_init();

	return 1;
}


int motors_pwm_init(){
	uint32_t period = 1000; /* PWM period in microseconds -> 1kHz*/

	PWM_Params_init(&pwm5_params);
	pwm5_params.period = period;             	// Period in microseconds
	pwm5_params.dutyMode = PWM_DUTY_SCALAR; 	// duty is an integer scaled to the period,
										  	  // 0 = 0% and 65535 = 100%

	PWM_Params_init(&pwm6_params);
	pwm6_params.period = period;             	// Period in microseconds
	pwm6_params.dutyMode = PWM_DUTY_SCALAR; 	// duty is an integer scaled to the period,
											  	  // 0 = 0% and 65535 = 100%
	PWM_Params_init(&pwm7_params);
	pwm7_params.period = period;             	// Period in microseconds
	pwm7_params.dutyMode = PWM_DUTY_SCALAR; 	// duty is an integer scaled to the period,
											  // 0 = 0% and 65535 = 100%
	PWM_Params_init(&pwm8_params);
	pwm8_params.period = period;             	// Period in microseconds
	pwm8_params.dutyMode = PWM_DUTY_SCALAR; 	// duty is an integer scaled to the period,
												  // 0 = 0% and 65535 = 100%

	pwm5_handle = PWM_open(Board_M5_EN_PWM, &pwm5_params);
	if (pwm5_handle == NULL) {
		return 0;
	}
	pwm6_handle = PWM_open(Board_M6_EN_PWM, &pwm6_params);
	if (pwm6_handle == NULL) {
		return 0;
	}
	pwm7_handle = PWM_open(Board_M7_EN_PWM, &pwm7_params);
	if (pwm7_handle == NULL) {
		return 0;
	}
	pwm8_handle = PWM_open(Board_M8_EN_PWM, &pwm8_params);
	if (pwm8_handle == NULL) {
		return 0;
	}

	return 1;
}

/*
 * arguments are integers scaled to the speed 0 = 0% and 32767 = 100% and the sign of the number
 * determines the direction of travel (negative: backwards, positive: forward).
 */
void motors_wheels_move(int16_t front_left, int16_t front_right, int16_t rear_left, int16_t rear_right)
{
	GPIO_write(Board_M5678_CURR_SENS_EN, 1); //turn current sensors on

	GPIO_write(Board_M5678_SLEEP_N, 1); //turn h-bridge on

	/* determine direction of the wheel (forward/backward) */
	if(front_left < 0)
	{
		GPIO_write(Board_M5_PH, 0); //set phase
		front_left = -front_left;
	}
	else
		GPIO_write(Board_M5_PH, 1); //set phase

	if(front_right < 0)
	{
		GPIO_write(Board_M6_PH, 0); //set phase
		front_right = -front_right;
	}
	else
		GPIO_write(Board_M6_PH, 1); //set phase

	if(rear_left < 0)
	{
		GPIO_write(Board_M7_PH, 0); //set phase
		rear_left = -rear_left;
	}
	else
		GPIO_write(Board_M7_PH, 1); //set phase

	if(rear_right < 0)
	{
		GPIO_write(Board_M8_PH, 0); //set phase
		rear_right = -rear_right;
	}
	else
		GPIO_write(Board_M8_PH, 1); //set phase

	PWM_setDuty(pwm5_handle, (front_left));
	PWM_setDuty(pwm6_handle, (front_right));
	PWM_setDuty(pwm7_handle, (rear_left));
	PWM_setDuty(pwm8_handle, (rear_right));
}


void motors_wheels_update_distance()
{

	static uint16_t sensor_values[N_WHEELS];

	sensor_values[0] = 0;
	sensor_values[1] = 0;
	sensor_values[2] = 0;
	sensor_values[3] = 0;

	adc_read_motor_sensors(sensor_values);

}


/*
 * arguments determine the direction of the struts movement
 * (negative: backwards, positive: forward, zero: stop).
 */
void motors_struts_move(int8_t front_left, int8_t front_right, int8_t rear_left, int8_t rear_right)
{
	GPIO_write(Board_M1234_SLEEP_N, 1); //turn h-bridge on

	if(front_left < 0)
		GPIO_write(Board_M1_PH, 0); //set phase backward
	else
		GPIO_write(Board_M1_PH, 1); //set phase forward

	if(front_right < 0)
		GPIO_write(Board_M2_PH, 0);
	else
		GPIO_write(Board_M2_PH, 1);

	if(rear_left < 0)
		GPIO_write(Board_M3_PH, 0);
	else
		GPIO_write(Board_M3_PH, 1);

	if(rear_right < 0)
		GPIO_write(Board_M4_PH, 0); //set phase
	else if(rear_right > 0)
		GPIO_write(Board_M4_PH, 1); //set phase

	/* if value is zero, motor should be disabled, else enabled */
	GPIO_write(Board_M1_EN, (front_left));
	GPIO_write(Board_M2_EN, (front_right));
	GPIO_write(Board_M3_EN, (rear_left));
	GPIO_write(Board_M4_EN, (rear_right));
}


void motors_wheels_stop()
{
	PWM_setDuty(pwm5_handle, 0);
	PWM_setDuty(pwm6_handle, 0);
	PWM_setDuty(pwm7_handle, 0);
	PWM_setDuty(pwm8_handle, 0);

	GPIO_write(Board_M5678_CURR_SENS_EN, 0); //turn current sensor off
	GPIO_write(Board_M5678_SLEEP_N, 1); //turn h-bridge off
}


void motors_struts_stop()
{
	GPIO_write(Board_M1_EN, 0);
	GPIO_write(Board_M2_EN, 0);
	GPIO_write(Board_M3_EN, 0);
	GPIO_write(Board_M4_EN, 0);

	GPIO_write(Board_M1234_SLEEP_N, 1); //turn h-bridge off
}


/* after having called this function, the motor_init function needs to be called again. */
void motors_pwm_close()
{
	PWM_close(pwm5_handle); //turns off the pwm signal
	PWM_close(pwm6_handle); //turns off the pwm signal
	PWM_close(pwm7_handle); //turns off the pwm signal
	PWM_close(pwm8_handle); //turns off the pwm signal
}
