/*
 * motors.c
 *
 *  Created on: 13 Oct 2015
 *      Author: raffael
 */

#include "motors.h"
#include "../../../Board.h"


int motors_init(){
    GPIO_write(Board_M5678_SLEEP_N, 1); //turn h-bridge on
    GPIO_write(Board_M5_PH, 1); //set phase
    GPIO_write(Board_M6_PH, 1); //set phase
    GPIO_write(Board_M7_PH, 1); //set phase
    GPIO_write(Board_M8_PH, 1); //set phase

	PWM_Handle pwm5_handle, pwm6_handle, pwm7_handle, pwm8_handle;
	PWM_Params pwm5_params, pwm6_params, pwm7_params, pwm8_params;

	uint32_t period = 10000; /* PWM period in microseconds -> 100Hz*/
	uint32_t duty = 32000; // duty is an integer scaled to the period,
	  // 0 = 0% and 65535 = 100%

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

	PWM_setDuty(pwm5_handle, duty);
	PWM_setDuty(pwm6_handle, duty);
	PWM_setDuty(pwm7_handle, duty);
	PWM_setDuty(pwm8_handle, duty);


	//wait for the bit to "go through"
	Task_sleep(2000);

	Task_sleep(20000);

	PWM_close(pwm5_handle); //turns off the pwm signal

	return 1;
}
