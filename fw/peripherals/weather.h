/*
 *  File: weather.h
 *  Description: High level model for accessing calibrated temperature, pressure and moisture
 *  Author:
 */



//in 0.01 degree Centigrade
int weather_get_int_temp();

//in Pa
unsigned int weather_get_int_press();

//value of 42313 represents 42313 / 1024 = 41.321 %rH
unsigned int weather_get_int_humid();

//in 0.01 degree Centigrade
int weather_get_ext_temp();

//in Pa
unsigned int weather_get_ext_press();

//value of 42313 represents 42313 / 1024 = 41.321 %rH
unsigned int weather_get_ext_humid();


uint32_t weather_get_uv_light();


uint32_t weather_get_ir_light();


float weather_get_vis_lux();


float weather_get_irradiance();

void weather_task();
