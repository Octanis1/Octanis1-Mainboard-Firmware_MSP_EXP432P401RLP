/*
 * imu.c
 *
 *  Created on: 19 Aug 2015
 *      Author: Sam
 */
#if defined(NAVIGATION_TEST)
#else
#include "../../Board.h"
#endif
#include "imu.h"
#include "hal/motors.h"
#include "hal/bno055_support.h"
#include "hal/i2c_helper.h"

//mavlink includes
#include "gps.h"
#include "comm.h"
#include "hal/time_since_boot.h"

#define M_PI 3.14159265358979323846
#define T_IMU_TASK 		100
#define T_IMU_INVALID	5000 //time after which the rover shall consider imu data to be invalid.

static struct _imu_data {
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
	double d_euler_data_p;
	double d_euler_data_h; // from 0 to 360deg
	double d_euler_data_r;
	unsigned char calib_status;
	uint32_t t_since_last_imu_update_ms; // is the rover already/still receiving valid IMU information?
} imu_data;

bool imu_valid()
{
#ifdef USE_ONBOARD_BNO055
	return (imu_data.t_since_last_imu_update_ms < T_IMU_INVALID) && imu_data.calib_status > 6;
#else
	return (imu_data.t_since_last_imu_update_ms < T_IMU_INVALID);
#endif
}

// contains the data from an external IMU (f.ex. the one connected to the SBC)
static mavlink_attitude_t imu_attitude;

// pitch Euler data in 100 degrees
int16_t imu_get_pitch(){
	return (int16_t)(100*imu_data.d_euler_data_p);
}

// heading Euler data in centi degrees
int16_t imu_get_heading(){
	return (int16_t)(100*imu_data.d_euler_data_h);
}

float imu_get_fheading(){
	return (float)imu_data.d_euler_data_h;
}

// roll Euler data in 100 degrees
int16_t imu_get_roll(){
	return (int16_t)(100*imu_data.d_euler_data_r);
}

// return IMU calib status
uint8_t imu_get_calib_status(){
#ifdef USE_ONBOARD_BNO055
	return (uint8_t)(imu_data.calib_status);
#else
	return 9;
#endif
}

// linear acceleration data in 100 m/s^2
int16_t imu_get_accel_x(){
	return (imu_data.accel_x);
}

// linear acceleration data in 100 m/s^2
int16_t imu_get_accel_y(){
	return (imu_data.accel_y);
}

// linear acceleration data in 100 m/s^2
int16_t imu_get_accel_z(){
	return (imu_data.accel_z);
}

void imu_update_attitude_from_mavlink(mavlink_message_t* msg){
	mavlink_msg_attitude_decode(msg, &imu_attitude);
#ifndef USE_ONBOARD_BNO055
	//note: IMU is placed in a way that roll and pitch are inversed and heading is mirrored.
	imu_data.d_euler_data_p = -(double)(180/M_PI * imu_attitude.roll);
	imu_data.d_euler_data_r = -(double)(180/M_PI * imu_attitude.pitch);
	imu_data.d_euler_data_h = -(double)(180/M_PI * imu_attitude.yaw);
	if(imu_data.d_euler_data_h < 0.0)
		imu_data.d_euler_data_h = imu_data.d_euler_data_h + 360.0;

	imu_data.t_since_last_imu_update_ms = 0;
#endif
}

COMM_FRAME* imu_pack_mavlink_attitude()
{
	// Initialize the message buffer
	static COMM_FRAME frame;

#ifdef USE_ONBOARD_BNO055
	float roll = (float) imu_get_roll()/100;
	float roll_deg = roll*M_PI/180;
	float pitch = (float) imu_get_pitch()/100;
	float pitch_deg = pitch*M_PI/180;
	float yaw = imu_get_fheading();
	float yaw_rad = deg2rad(yaw);
	float rollspeed = 0.; //TODO
	float pitchspeed = 0.; //TODO
	float yawspeed = 0.; //TODO

	// Pack the message
	mavlink_msg_attitude_pack(mavlink_system.sysid, MAV_COMP_ID_IMU, &(frame.mavlink_message), // ROLL AND PITCH DO NOT UPDATE ON AMP PLANNER
			ms_since_boot(), roll_deg, pitch_deg, yaw_rad, rollspeed, pitchspeed, yawspeed);
#else
	if(imu_attitude.time_boot_ms == 0) // no attitude information received yet
		return NULL;

	// just re-send the previously received message.
	mavlink_msg_attitude_encode(mavlink_system.sysid, MAV_COMP_ID_IMU, &(frame.mavlink_message),
			&imu_attitude);
#endif

	return &frame;
}

float deg2rad(float deg)
{
	float rad;
	if(deg>=0 && deg<180) rad = deg*M_PI/180;
	else rad = (deg-360)*M_PI/180;
	return rad;
}

void imu_task(){
	imu_data.t_since_last_imu_update_ms = T_IMU_INVALID;

	i2c_helper_init_handle();

	Task_sleep(500);

	cli_init();

#ifdef USE_ONBOARD_BNO055
	imu_init();
#endif

	while(1){
#ifdef USE_ONBOARD_BNO055
		imu_data.calib_status=bno055_check_calibration_status(); //this line alone lets the i2c bus crash
		if(bno055_get_heading(&(imu_data.d_euler_data_h), &(imu_data.d_euler_data_p), &(imu_data.d_euler_data_r)) == SUCCESS) //this line alone lets the i2c bus crash
			imu_data.t_since_last_imu_update_ms = 0;

//		bno055_get_accel(&(imu_data.accel_x), &(imu_data.accel_y), &(imu_data.accel_z)); //commenting out this line alone still lets the i2c bus be blocked
#endif


		COMM_FRAME* imu_frame = imu_pack_mavlink_attitude();
		if(imu_frame!=NULL)
			comm_mavlink_broadcast(imu_frame);


	//	if(calib_status > 8)
	//	{
	//
	//		GPIO_write(Board_LED_RED, Board_LED_OFF);
	//	}
	//	else
	//	{
	//		GPIO_write(Board_LED_RED, Board_LED_ON);
	//
	//	}


	//	if(d_euler_data_h > 180)
	//	{
	//		GPIO_write(Board_LED_GREEN, Board_LED_ON);
	//	}
	//	else
	//	{
	//		GPIO_write(Board_LED_GREEN, Board_LED_OFF);
	//	}

		//	bno055_data_readout_template();



		/*
		if(drive_get_status() == NOT_MOVING){
			//motion interrupts to process?
			// block here if we're not driving.
		}*/


			// BNO055 debug
			//serial_printf(cli_stdout, "BNO begin \n", 0);

		//	int b = bno055_begin(BNO055_MAIN, i2c_helper_handle);

		//	serial_printf(cli_stdout, "bno startup %d \n", b);


		//only check every 10 seconds
//		static uint8_t i_since_last_measurement = 0;
//		if(!(i_since_last_measurement++ % 20))
//		motors_struts_get_position();

		Task_sleep(T_IMU_TASK);
		imu_data.t_since_last_imu_update_ms += T_IMU_TASK;


	}



}

