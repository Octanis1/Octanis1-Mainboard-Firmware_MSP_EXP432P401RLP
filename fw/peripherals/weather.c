/*
 *  File: weather.c
 *  Description: High level model for accessing calibrated temperature, pressure and moisture
 *  Author:
 */
#include "../../Board.h"

#include "weather.h"
#include "hal/bmp180.h"
#include "../lib/printf.h"

void weather_task(){
	Task_sleep(6000);


	//DEBUGGING BMP180 START
	cli_printf("w begin \n", 0);
	bmp180_begin();
	float temp = bmp180_get_temp();

	cli_printf("temp: %d \n", temp);

	cli_printf("w end \n", 0);
	//DEBUGGING BMP180 END




}
