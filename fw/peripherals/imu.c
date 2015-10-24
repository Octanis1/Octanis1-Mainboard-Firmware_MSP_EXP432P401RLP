/*
 * imu.c
 *
 *  Created on: 19 Aug 2015
 *      Author: Sam
 */
#include "../../Board.h"
#include "imu.h"
#include "hal/motors.h"
#include "hal/bno055_support.h"
#include "hal/i2c_helper.h"

void imu_task(){

	i2c_helper_init_handle();
//	cli_printf("BNO begin \n", 0);
	imu_init();
	motors_init();


	double d_euler_data_p;
	double d_euler_data_h;
	unsigned char calib_status;

	while(1){

	calib_status=bno055_check_calibration_status();
	if(calib_status > 8)
	{
		GPIO_write(Board_LED_RED, Board_LED_OFF);
	}
	else
	{
		GPIO_write(Board_LED_RED, Board_LED_ON);
	}


	bno055_get_heading(&d_euler_data_h, &d_euler_data_p);
	if(d_euler_data_h > 180)
	{
		GPIO_write(Board_LED_GREEN, Board_LED_ON);
	}
	else
	{
		GPIO_write(Board_LED_GREEN, Board_LED_OFF);
	}

	//	bno055_data_readout_template();

	/*
	if(drive_get_status() == NOT_MOVING){
		//motion interrupts to process?
		// block here if we're not driving.
	}*/


		// BNO055 debug
		//cli_printf("BNO begin \n", 0);

	//	int b = bno055_begin(BNO055_MAIN, i2c_helper_handle);

	//	cli_printf("bno startup %d \n", b);

		Task_sleep(500);

	}



}
