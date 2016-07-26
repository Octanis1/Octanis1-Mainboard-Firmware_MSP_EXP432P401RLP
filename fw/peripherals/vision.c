/*
 * vision.c
 *
 *  Created on: Sep 12, 2015
 *      Author: vagrant
 */

#include "vision.h"

#include "hal/ultrasonic.h"
#include "../../Board.h"
#include "../core/eps.h"


//TODO: remove!!!
#include "hal/i2c_helper.h"
#define MotorSpeedSet             0x82
#define PWMFrequenceSet           0x84
#define DirectionSet              0xaa
#define MotorSetA                 0xa1
#define MotorSetB                 0xa5
#define Nothing                   0x01
#define EnableStepper             0x1a
#define UnenableStepper           0x1b
#define Stepernu                  0x1c
#define I2CMotorDriverAdd         0x0f   // Set the address of the I2CMotorDriver
void pwm_i2c_test()
{
	static unsigned char speed = 0;
	static unsigned char freq = 0;
	speed = speed + 50;
	 write8(I2CMotorDriverAdd, MotorSpeedSet);
	 write8(I2CMotorDriverAdd, speed);
	 write8(I2CMotorDriverAdd, speed);

	 freq++;
	 write8(I2CMotorDriverAdd, PWMFrequenceSet);
	 write8(I2CMotorDriverAdd, freq);
	 write8(I2CMotorDriverAdd, freq);


}


void vision_task(){
	cli_init();

//	ultrasonic_init();
	eps_init();

	int32_t distance_values[N_ULTRASONIC_SENSORS_PER_ARRAY*N_ULTRASONIC_ARRAYS];
	int8_t directions_result[N_ULTRASONIC_SENSORS_PER_ARRAY*N_ULTRASONIC_ARRAYS] = {(int8_t) 1}; //all ok


	while(1){

//		eps_switch_module(M3V3_1_ON);
//		if(ultrasonic_get_distance(distance_values))
//		{
//		//successfully read the sensor values
////			serial_printf(cli_stdout, 'us check %d \n', (int)CRITICAL_DISTANCE_THRESHOLD_TIMESTAMP);
//			ultrasonic_check_distance(distance_values, directions_result);
//		}
//		else
//		{
//		//not all or none of the sensors returned a pulse
//		}
		Task_sleep(5000);

		pwm_i2c_test();

//		eps_switch_module(M3V3_1_OFF);

	}

}
