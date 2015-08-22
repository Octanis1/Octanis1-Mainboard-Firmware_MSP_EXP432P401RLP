/*
 * imu.c
 *
 *  Created on: 19 Aug 2015
 *      Author: Sam
 */
#include "../../Board.h"
#include "imu.h"
#include "hal/bno055.h"
#include "hal/i2c_helper.h"

void imu_task(){

	//setup imu
	//I2C_Handle i2c_handle = i2c_helper_get_handle();
	/* Initialise I2C Bus */
			/* I2C_Params      params;
			I2C_Params_init(&params);
			i2c_helper_handle = I2C_open(Board_I2C0, &params);

			if (!i2c_helper_handle) {
				cli_printf("I2C did not or already open\n", 0);
			}
			*/

	while(1){

		/*
		if(drive_get_status() == NOT_MOVING){
			//motion interrupts to process?
			// block here if we're not driving.
		}*/


		// BNO055 debug
		//cli_printf("BNO begin \n", 0);

	//	int b = bno055_begin(BNO055_MAIN, i2c_helper_handle);

	//	cli_printf("bno startup %d \n", b);

		Task_sleep(5000);

	}



}
