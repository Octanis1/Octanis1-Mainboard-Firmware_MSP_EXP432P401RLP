/*
 * imu.h
 *
 *  Created on: 19 Aug 2015
 *      Author: Sam
 */

#ifndef FW_PERIPHERALS_IMU_H_
#define FW_PERIPHERALS_IMU_H_

#include "../lib/mavlink/common/mavlink.h"

//to externally set the attitude information from incoming mavlink message
void imu_update_attitude_from_mavlink(mavlink_message_t* msg);

//
bool imu_valid();

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

float deg2rad(float deg);

void imu_task();


#endif /* FW_PERIPHERALS_IMU_H_ */
