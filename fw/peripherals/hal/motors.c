/*
 * motors.c
 *
 *  Created on: 13 Oct 2015
 *      Author: raffael
 */

#include "motors.h"
#include "../../../Board.h"
#include "adc.h"
#include "spi_helper.h"
#include "AS5050A.h"
#include "../../core/eps.h"
#include "time_since_boot.h"

#define INITIAL_FRICTION_FACTOR 1
#define BLOCKED_THRESHOLD		1
#define AIR_THRESHOLD			1
#define TURNING_CONSTANT		1
#define MISSION_LATTITUDE		0	//to be defined separately for each mission
#define Y_TO_LATTITUDE			30.81840482
#define KILO					1000
#define MAX_RECENT_VALUES		75
#define VALUES_AFTER_GPS_RESET  25

static float friction_factor = INITIAL_FRICTION_FACTOR;

void motors_distance_odometer(uint16_t sensor_values[N_WHEELS], float voltage[N_WHEELS], float heading);
int motors_navigation_error(uint16_t sensor_values[N_WHEELS], float voltage[N_WHEELS]);

PWM_Handle pwm5_handle, pwm6_handle, pwm7_handle, pwm8_handle;
PWM_Params pwm5_params, pwm6_params, pwm7_params, pwm8_params;

struct odometer{
	float right_left[MAX_RECENT_VALUES]; //stores all recent values for x
	float up_down[MAX_RECENT_VALUES];	  //stores all recent values for y
	int i;
	float x;   //in m
	float y;   //in m
	float lat; //in seconds
	float lon; //in seconds
	int first_time; //true/false
	int gps_time; //in msec
	int odo_time; //in msec
} odometer;

int motors_init()
{
//	eps_init();
	motors_pwm_init();
	adc_init();
//	spi_helper_init_handle();

	return 1;
}

int motors_pwm_init(){
	uint32_t period = PWM_PERIOD; /* PWM period in microseconds -> 50kHz*/

	PWM_Params_init(&pwm5_params);
	pwm5_params.period = period;             	// Period in microseconds
	pwm5_params.dutyMode = PWM_DUTY_TIME; 	// duty is an integer scaled to the period,
										  	  // 0 = 0% and 65535 = 100%
	PWM_Params_init(&pwm6_params);
	pwm6_params.period = period;             	// Period in microseconds
	pwm6_params.dutyMode = PWM_DUTY_TIME; 		// duty is an integer scaled to the period,
											// 0 = 0% and 65535 = 100%
	PWM_Params_init(&pwm7_params);
	pwm7_params.period = period;             	// Period in microseconds
	pwm7_params.dutyMode = PWM_DUTY_TIME; 	// duty is an integer scaled to the period,
											  // 0 = 0% and 65535 = 100%
	PWM_Params_init(&pwm8_params);
	pwm8_params.period = period;             	// Period in microseconds
	pwm8_params.dutyMode = PWM_DUTY_TIME; 	// duty is an integer scaled to the period,
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
 * arguments are integers scaled to the speed 0 = 0% and 65535 = 100% and the sign of the number
 * determines the direction of travel (negative: backwards, positive: forward).
 */
void motors_wheels_move(int32_t front_left, int32_t front_right, int32_t rear_left, int32_t rear_right)
{
	serial_printf(cli_stdout, "mv %d %d %d %d\n\r",front_left, front_right, rear_left, rear_right);


	eps_switch_module(M11V_ON);

	GPIO_write(Board_M5678_CURR_SENS_EN, 1); //turn current sensors on

	GPIO_write(Board_M5678_ON, 1); //turn h-bridge on

	Task_sleep(20); //wait for vcc to ramp up

	/* determine direction of the wheel (forward/backward) */
	if(front_left < 0)
	{
		GPIO_write(Board_M5_IN2, PH_REVERSE);
		front_left = PWM_SPEED_100+front_left;
	}
	else	{
		GPIO_write(Board_M5_IN2, PH_FORWARD);}
	PWM_setDuty(pwm5_handle, (front_left));

	Task_sleep(50); //wait delay the other turn-on's to limit current peak
	if(front_right < 0)
	{
		GPIO_write(Board_M6_IN2, PH_REVERSE);
		front_right = PWM_SPEED_100+front_right;
	}
	else{
		GPIO_write(Board_M6_IN2, PH_FORWARD);}
	PWM_setDuty(pwm6_handle, (front_right));
	Task_sleep(50); //wait delay the other turn-on's to limit current peak

	if(rear_left < 0)
	{
		GPIO_write(Board_M7_IN2, PH_REVERSE);
		rear_left = PWM_SPEED_100+rear_left;
	}
	else{
		GPIO_write(Board_M7_IN2, PH_FORWARD);}
	PWM_setDuty(pwm7_handle, (rear_left));
	Task_sleep(50); //wait delay the other turn-on's to limit current peak

	if(rear_right < 0)
	{
		GPIO_write(Board_M8_IN2, PH_REVERSE);
		rear_right = PWM_SPEED_100+rear_right;
	}
	else{
		GPIO_write(Board_M8_IN2, PH_FORWARD);}
	PWM_setDuty(pwm8_handle, (rear_right));
}


int motors_wheels_update_distance(int32_t voltage[N_SIDES], float heading)
{
	static uint16_t sensor_values[N_WHEELS];

	/* initialize to 0 to reset the running average inside the adc readout function */
	sensor_values[0] = 0;
	sensor_values[1] = 0;
	sensor_values[2] = 0;
	sensor_values[3] = 0;

	adc_read_motor_sensors(sensor_values);

	//check to see if a wheel is blocked or off the ground
	if(motors_navigation_error(sensor_values, voltage)) {
		return 0;
	}
	else {
		//function to estimate the distance needs curvature.
		motors_distance_odometer(sensor_values, voltage, heading);
		return 1;
	}
}

int motors_navigation_error(uint16_t sensor_values[N_WHEELS], float voltage[N_WHEELS])
{
	int error = 0;
	int ratio[N_WHEELS];
	int i = 0;
	for (i=0; i<N_WHEELS; i++) {
		ratio[i] = sensor_values[i] / voltage[i%N_SIDES];
		if (ratio[i]>BLOCKED_THRESHOLD || ratio[i]<AIR_THRESHOLD) {
			error++;
			return 1;
		}
	}
	if (!error)
		return 0;
}

void motors_distance_odometer(uint16_t sensor_values[N_WHEELS], float voltage[N_WHEELS], float heading)
{
	//take values for sensors and convert to distance
	float distance_meters, circumference, velocity, delta_time = 0;
	float speed[N_WHEELS], rps[N_WHEELS];
	int i = 0;
	float x_to_longitude = Y_TO_LATTITUDE * cos(MISSION_LATTITUDE);

	circumference = WHEEL_RADIUS * 2 * M_PI;

	for (i=0; i<N_WHEELS; i++) {
		rps[i] = sensor_values[i] * friction_factor / voltage[i%N_SIDES];
		speed[i] = rps[i] * circumference;
		velocity += speed[i];
	}

	velocity = velocity / (N_WHEELS*KILO);

	uint32_t msec = ms_since_boot();

	if (odometer.first_time) {
		odometer.gps_time = msec;
		odometer.odo_time = msec;
		odometer.first_time = 0;
		odometer.i++;
	}
	else {
		delta_time = msec - odometer.odo_time;
		odometer.odo_time = msec;
		if (odometer.i == (MAX_RECENT_VALUES-1))
			odometer.i = VALUES_AFTER_GPS_RESET;
		else
			odometer.i++;
	}
	//only straight forward!
	distance_meters = delta_time * velocity;
	odometer.right_left[odometer.i] = sin(heading) * distance_meters;
	odometer.up_down[odometer.i] = cos(heading) * distance_meters;
	odometer.x += sin(heading) * distance_meters;
	odometer.y += cos(heading) * distance_meters;

	//turning left
	//odometer.x += sin(heading - 0.5 * M_PI) * distance_meters * TURNING_CONSTANT;
	//odometer.y += cos(heading - 0.5 * M_PI) * distance_meters * TURNING_CONSTANT;

	//turning right
	//odometer.x += sin(heading + 0.5 * M_PI) * distance_meters * TURNING_CONSTANT;
	//odometer.y += cos(heading + 0.5 * M_PI) * distance_meters * TURNING_CONSTANT;

	//convert distance to polar coordinates
	odometer.lon = odometer.x / x_to_longitude;
	odometer.lat = odometer.y / Y_TO_LATTITUDE;
}

void motors_gps_input(float delta_lat, float delta_lon)
{
	//recalibrate friction_factor
	float update = 0;
	update = ((delta_lat / odometer.lat)+(delta_lon / odometer.lon))/2;
	friction_factor = friction_factor * update;

	//reinitialize odometer_distance
	int i;
	float x_to_longitude = Y_TO_LATTITUDE * cos(MISSION_LATTITUDE);

	odometer.x = 0;
	odometer.y = 0;
	odometer.first_time = 1;

	for (i = 0; i<VALUES_AFTER_GPS_RESET; i++) //later values not reinitialised because not deemed necessary - possibly not the case!
	{
		odometer.right_left[i] = odometer.right_left[i + MAX_RECENT_VALUES - VALUES_AFTER_GPS_RESET];
		odometer.up_down[i] = odometer.up_down[i + MAX_RECENT_VALUES - VALUES_AFTER_GPS_RESET];

		odometer.x += odometer.right_left[i];
		odometer.y += odometer.up_down[i];
	}

	//convert distance to polar coordinates
	odometer.lon = odometer.x / x_to_longitude;
	odometer.lat = odometer.y / Y_TO_LATTITUDE;
}

/*
 * arguments determine the direction of the struts movement
 * (negative: backwards, positive: forward, zero: stop). ????????????
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


void motors_struts_get_position()
{
	static uint16_t motor_sensor_values[N_STRUTS];

	/* initialize to 0 to reset the running average inside the adc readout function */
	motor_sensor_values[0] = 0;
	motor_sensor_values[1] = 0;
	motor_sensor_values[2] = 0;
	motor_sensor_values[3] = 0;

//	adc_read_strut_sensor_values(motor_sensor_values);
//
//	//TODO: remove test output.
//	static uint16_t degrees;
//	degrees = motor_sensor_values[0] / (N_ADC_AVG_STRUT*11.378);
//
//	uint16_t angle = 0;
////	as5050_read_data(&angle);
//
//		//serial_printf(cli_stdout, "%u\n",degrees);


}

void motors_wheels_stop()
{
//	serial_printf(cli_stdout, "stop\n\r",0);

	PWM_setDuty(pwm5_handle, 0);
	PWM_setDuty(pwm6_handle, 0);
	PWM_setDuty(pwm7_handle, 0);
	PWM_setDuty(pwm8_handle, 0);

	GPIO_write(Board_M5_IN2, 0);
	GPIO_write(Board_M6_IN2, 0);
	GPIO_write(Board_M7_IN2, 0);
	GPIO_write(Board_M8_IN2, 0);

	GPIO_write(Board_M5678_CURR_SENS_EN, 0); //turn current sensor off
	GPIO_write(Board_M5678_ON, 0); //turn h-bridge off

//	eps_switch_module(M11V_OFF); // turn supply off TODO: as we connect the lidar motor to 11V, we dont turn it off.

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
