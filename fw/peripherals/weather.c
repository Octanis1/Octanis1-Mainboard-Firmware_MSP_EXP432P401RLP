/*
 *  File: weather.c
 *  Description: High level model for accessing calibrated temperature, pressure and moisture
 *  Author:
 */
#include "../../Board.h"

#include "weather.h"
#include "hal/AS3935.h"
#include "hal/bme280i2c.h"
#include "hal/bmp180i2c.h"
#include "hal/SHT2x.h"
#include "hal/windsensor.h"
#include "hal/mcp3425.h"
#include "hal/i2c_helper.h"
#include "../lib/printf.h"
#include "geiger.h"
#include "../core/log.h"


static struct _weather_data {
	int int_temp; //in 0.01 degree Centigrade
	unsigned int int_press; //in Pa
	unsigned int int_humid; //value of 42313 represents 42313 / 1024 = 41.321 %rH
	int ext_temp_bmp180; //in 0.1 degree Centigrade
	float ext_temp_sht21;
	int ext_temp; // average of both values in 0.01 degree Centigrade
	unsigned int ext_press; //pressure in steps of 1.0 Pa
	float ext_humid;
} weather_data;

//in 0.01 degree Centigrade
int weather_get_int_temp(){
	return weather_data.int_temp;
}

//in Pa
unsigned int weather_get_int_press(){
	return weather_data.int_press;
}

//value of 42313 represents 42313 / 1024 = 41.321 %rH
unsigned int weather_get_int_humid(){
	return weather_data.int_humid;
}

//in 0.01 degree Centigrade
int weather_get_ext_temp(){
	return weather_data.ext_temp;
}

//in Pa
unsigned int weather_get_ext_press(){
	return weather_data.ext_press;
}

//value of 42313 represents 42313 / 1024 = 41.321 %rH
unsigned int weather_get_ext_humid(){
	return (int)(1024*weather_data.ext_humid);
}

/* function calculating averages etc. of measured sensor data */
void weather_aggregate_data()
{
	// average SHT21 and BMP180 external temperature data and scale the result to 0.01 degree Celsius
	weather_data.ext_temp = (int)((10*weather_data.ext_temp_sht21+(float)weather_data.ext_temp_bmp180)*5);
}


void log_weather(struct _weather_data *d)
{
    struct logger *l = logger_get("weather");
    cmp_write_array(l->ctx, 8);
    cmp_write_integer(l->ctx, d->int_temp);
    cmp_write_uinteger(l->ctx, d->int_press);
    // ...
    logger_finish(l);
}

void weather_task(){

	i2c_helper_init_handle();
	windsensor_init();
//	mcp_init();
	geiger_turn_on_off(ON);

	lightning_reset();
	lightning_calibrate();



	while(1){
		Task_sleep(3000);
		bmp180_data_readout_template(&(weather_data.ext_temp_bmp180),&(weather_data.ext_press));

		bme280_data_readout_template(&(weather_data.int_temp),&(weather_data.int_press),&(weather_data.int_humid));
		//note: bme280 can give pressure, humidity and temperature



//		float uv = mcp_get_data();

//		weather_data.ext_temp_sht21 = sht2x_get_temp(); //TODO: fix the fact that program stops here if sensor is not connected.
//		weather_data.ext_humid = sht2x_get_humidity();

		//	windsensor_getvalue();

		weather_aggregate_data();

        log_weather(&weather_data);
	}
}
