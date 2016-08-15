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
#include <math.h>

#define INITIAL_FRICTION_FACTOR  1  //initialization constants revised by system after each GPS update
#define INITIAL_TURNING_CONSTANT 1
#define INITIAL_DISTANCE_CONST	 1
#define INITIAL_ANGLE_CONSTANT   1

#define BLOCKED_THRESHOLD		 1  //defined by user
#define AIR_THRESHOLD			 1
#define MISSION_LATITUDE		 0

#define Y_TO_LATITUDE			 110946.257352 //true constants
#define KILO					 1000
#define WHEEL_DISTANCE			 0.44

#define SENSOR_VALUES_UNAVAILABLE

static float friction_factor = INITIAL_FRICTION_FACTOR;
static float angle_constant = INITIAL_ANGLE_CONSTANT;
float x_to_longitude = Y_TO_LATITUDE * cos(MISSION_LATITUDE);

void motors_distance_odometer(uint16_t sensor_values[N_WHEELS], int32_t voltage[N_WHEELS], int position_i);
void motors_update_xy(int position_i);
void motors_get_radius_and_angle(float speed[N_WHEELS], float delta_time, int position_i);
int motors_navigation_error(uint16_t sensor_values[N_WHEELS], int32_t voltage[N_WHEELS]);

PWM_Handle pwm5_handle, pwm6_handle, pwm7_handle, pwm8_handle;
PWM_Params pwm5_params, pwm6_params, pwm7_params, pwm8_params;

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


int motors_run_odometer(int32_t voltage[N_SIDES], int position_i)
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
		motors_distance_odometer(sensor_values, voltage, position_i);
		return 1;
	}
}

int motors_navigation_error(uint16_t sensor_values[N_WHEELS], int32_t voltage[N_WHEELS])
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

void motors_distance_odometer(uint16_t sensor_values[N_WHEELS], int32_t voltage[N_WHEELS], int position_i)
{
	float circumference, velocity, delta_time = 0;
	float speed[N_WHEELS], rps[N_WHEELS];
	int i = 0;
	uint32_t msec;

	circumference = WHEEL_RADIUS * 2 * M_PI;

#ifdef SENSOR_VALUES_AVAILABLE
	for (i=0; i<N_WHEELS; i++) {
		rps[i] = friction_factor * voltage[i%N_SIDES] / sensor_values[i];
		speed[i] = rps[i] * circumference;
		velocity += speed[i];
	}
#endif
#ifdef SENSOR_VALUES_UNAVAILABLE
	for (i=0; i<N_WHEELS; i++) {
		rps[i] = friction_factor * voltage[i%N_SIDES];
		speed[i] = rps[i] * circumference;
		velocity += speed[i];
	}
#endif

	velocity = velocity / (N_WHEELS*KILO);
	odo.velocity = velocity * KILO;
	msec = ms_since_boot();
	delta_time = msec - odo.odo_time;
	odo.odo_time = msec;
	motors_get_radius_and_angle(speed, delta_time, position_i);
	odo.distance[position_i] = delta_time * velocity;
	motors_update_xy(position_i);
	odo.x += odo.right_left[position_i];
	odo.y += odo.up_down[position_i];

	//convert distance to polar coordinates
	odo.longitude = odo.x / x_to_longitude;
	odo.latitude = odo.y / Y_TO_LATITUDE;
}

void motors_update_xy(int position_i)
{
	odo.right_left[position_i] = odo.radius[position_i] - odo.radius[position_i] * cos(odo.angle[position_i]);
	odo.up_down[position_i] = odo.radius[position_i] * sin(odo.angle[position_i]);
	odo.x += odo.right_left[position_i];
	odo.y += odo.up_down[position_i];
	odo.heading += odo.angle[position_i];
}

void motors_get_radius_and_angle(float speed[N_WHEELS], float delta_time, int position_i)
{
	float radius, angle;

	radius = (WHEEL_DISTANCE / 2) * (speed[0] + speed[1])/(speed[0] - speed[1]);
	angle = speed[0] * delta_time * M_PI / (radius + WHEEL_DISTANCE / 2);

	odo.radius[position_i] = radius;
	odo.angle[position_i] = angle * angle_constant;
}

void motors_recalibrate_odometer(float delta_lat, float delta_lon, float delta_heading)
{
	//recalibrate friction_factor
	float update_friction, x_sum, y_sum, angle_sum = 0;
	int i, j;
	for (i = 0; i < (MAX_RECENT_VALUES - VALUES_AFTER_GPS_RESET); i++)
	{
		x_sum += odo.right_left[i];
		y_sum += odo.up_down[i];
	}
	odo.checked_lat = x_sum / x_to_longitude;
	odo.checked_lon = y_sum / Y_TO_LATITUDE;
	update_friction = sqrt(odo.checked_lat * odo.checked_lat + odo.checked_lon * odo.checked_lon)/sqrt(delta_lat * delta_lat + delta_lon * delta_lon); //look at units of delta_lat and delta_lon!!!
	friction_factor = friction_factor / update_friction;

	//recalibrate angle_constant
	for (j = 0; j < (MAX_RECENT_VALUES - VALUES_AFTER_GPS_RESET); j++)
	{
		angle_sum += odo.angle[j];
	}
	angle_constant = delta_heading / angle_sum;
}

void motors_reinitialize_odometer(float gps_heading)
{
	int i;
	odo.x = 0;
	odo.y = 0;
	odo.first_time = 1;

	//initialize x, y, lat, lon, heading
	for (i = 0; i < VALUES_AFTER_GPS_RESET; i++) //later values not reinitialised because not deemed necessary - possibly not the case!
	{
		odo.right_left[i] = odo.right_left[i + MAX_RECENT_VALUES - VALUES_AFTER_GPS_RESET];
		odo.up_down[i] = odo.up_down[i + MAX_RECENT_VALUES - VALUES_AFTER_GPS_RESET];
		odo.x += odo.right_left[i];
		odo.y += odo.up_down[i];
		odo.radius[i] = odo.radius[i + MAX_RECENT_VALUES - VALUES_AFTER_GPS_RESET];
		odo.angle[i] = odo.angle[i + MAX_RECENT_VALUES - VALUES_AFTER_GPS_RESET];
		odo.heading += odo.angle[i];
	}
	odo.longitude = odo.x / x_to_longitude;
	odo.latitude = odo.y / Y_TO_LATITUDE;
	odo.heading = odo.heading + gps_heading;
}

float motors_get_latitude()
{
	float latitude;
	latitude = odo.latitude;
	return latitude;
}

float motors_get_longitude()
{
	float longitude;
	longitude = odo.longitude;
	return longitude;
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

float motors_get_groundspeed()
{
	return odo.velocity;
}

void motors_initialize()
{
	int i;
	odo.x = 0;
	odo.y = 0;
	odo.latitude = 0;
	odo.longitude = 0;
	odo.checked_lat = 0;
	odo.checked_lon = 0;
	odo.old_gps_heading = 0;
	odo.odo_time = 0;
	odo.velocity = 0;
	for (i = 0; i < MAX_RECENT_VALUES; i++)
	{
		odo.distance[i] = 0;
		odo.angle[i] = 0;
		odo.radius[i] = 0;
		odo.right_left[i] = 0;
		odo.up_down[i] = 0;
	}
}
