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


void weather_task();




/*************** IMU stuff moved here ******************/

// pitch Euler data in 100 degrees
int16_t imu_get_pitch();

// heading Euler data in 100 degrees
int16_t imu_get_heading();

// heading Euler data
float imu_get_fheading();


// roll Euler data in 100 degrees
int16_t imu_get_roll();

// return IMU calib status
uint8_t imu_get_calib_status();

// linear acceleration data in 100 m/s^2
int16_t imu_get_accel_x();
// linear acceleration data in 100 m/s^2
int16_t imu_get_accel_y();
// linear acceleration data in 100 m/s^2
int16_t imu_get_accel_z();

/*************** END IMU stuff  ******************/
