/*
 * imu.h
 *
 *  Created on: 19 Aug 2015
 *      Author: Sam
 */

#ifndef FW_PERIPHERALS_IMU_H_
#define FW_PERIPHERALS_IMU_H_


// pitch Euler data in 100 degrees
int16_t imu_get_pitch();

// heading Euler data in 100 degrees
int16_t imu_get_heading();

// heading Euler data
float imu_get_fheading();

int16_t imu_get_accel_x();
int16_t imu_get_accel_y();
int16_t imu_get_accel_z();


// roll Euler data in 100 degrees
int16_t imu_get_roll();

// return IMU calib status
uint8_t imu_get_calib_status();


void imu_task();


#endif /* FW_PERIPHERALS_IMU_H_ */
