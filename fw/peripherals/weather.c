/*
 *  File: weather.c
 *  Description: High level model for accessing calibrated temperature, pressure and moisture
 *  Author:
 */
#include "../../Board.h"

#include "weather.h"
#include "hal/bmp180.h"
#include "hal/i2c_helper.h"
#include "../lib/printf.h"



void weather_task(){
	/* Initialise I2C Bus */
/*
		I2C_Params      params;
		I2C_Params_init(&params);
		i2c_helper_handle = I2C_open(Board_I2C0, &params);

		if (!i2c_helper_handle) {
			cli_printf("I2C did not open \n", 0);
		}
*/
	/* Initialise I2C Bus */
		I2C_Params      params;
		I2C_Params_init(&params);
		i2c_helper_handle = I2C_open(Board_I2C0, &params);

		if (!i2c_helper_handle) {
			cli_printf("I2C did not or already open\n", 0);
		}



	while(1){
		Task_sleep(30000); //replace with a semaphore pend on i2c bus


		//DEBUGGING BMP180 START
		/* bmp180_begin(i2c_handle);

		float inside_temperature = bmp180_get_temp(i2c_handle);
		float inside_pressure = bmp180_get_pressure(i2c_handle);

		cli_printf("cabin temp: %d \n", inside_temperature);
		cli_printf("cabin pressure: %d \n", inside_pressure);
	*/

		//DEBUGGING BMP180 END

	}
}
