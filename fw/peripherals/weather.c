/*
 *  File: weather.c
 *  Description: High level model for accessing calibrated temperature, pressure and moisture
 *  Author:
 */
#include "../../Board.h"

#include "weather.h"
#include "hal/bme280i2c.h"
#include "hal/bmp180i2c.h"
#include "hal/SHT2x.h"
#include "hal/windsensor.h"
#include "hal/i2c_helper.h"
#include "../lib/printf.h"



void weather_task(){

	i2c_helper_init_handle();
	windsensor_init();

	while(1){
		Task_sleep(3000);

	//	windsensor_getvalue();
		bme280_data_readout_template();

//		float inside_temperature = sht2x_get_temp();
//		float inside_humidity = sht2x_get_humidity();

		//TODO: write little routine that compare the value given by the different captors

		//note: bme280 can give pressure, humidity and temperature

		//cli_printf("temp: %f \n", &inside_temperature);
		//cli_printf("RH: %f \n", &inside_humidity);

		//bmp180_data_readout_template();
	//	bmp180_data_readout_template();

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
