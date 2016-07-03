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


//TODO:remove!
void weather_call_bme_from_other_task();


void weather_task();
