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
#include "../peripherals/comm.h"
#include "../lib/mavlink/common/mavlink.h"

static struct _imu_data {
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
	double d_euler_data_p;
	double d_euler_data_h;
	double d_euler_data_r;
	unsigned char calib_status;
} imu_data;

// pitch Euler data in 100 degrees
int16_t imu_get_pitch(){
	return (int16_t)(100*imu_data.d_euler_data_p);
}

// heading Euler data in 100 degrees
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
	return (uint8_t)(imu_data.calib_status);
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


COMM_FRAME* imu_pack_mavlink_raw_int()
{
	// Mavlink heartbeat
	// Define the system type, in this case an airplane
	int32_t lat = (int32_t)(10000000.0 * gps_get_lat()); //Latitude (WGS84), in degrees * 1E7
	int32_t lon = (int32_t)(10000000.0 * gps_get_lon()); //Longitude (WGS84), in degrees * 1E7
	int32_t alt = (int32_t)(1000.0 * gps_get_int_altitude());//Altitude (AMSL, NOT WGS84), in meters * 1000 (positive for up). Note that virtually all GPS modules provide the AMSL altitude in addition to the WGS84 altitude.
	uint32_t msec = 1000 * (uint32_t)Seconds_get();
	int32_t relative_alt = 0; //TODO!
	int16_t vx = 0; //TODO
	int16_t vy = 0; //TODO
	int16_t vz = 0; //TODO

	// Initialize the message buffer
	static COMM_FRAME frame;

	// Pack the message
	mavlink_msg_global_position_int_pack(mavlink_system.sysid, MAV_COMP_ID_IMU, &(frame.mavlink_message),
			msec, lat, lon, alt, relative_alt, vx, vy, vz,imu_get_heading());


	return &frame;
}

void imu_task(){
	/************* IMU STUFF moved here *************/
	i2c_helper_init_handle();

	Task_sleep(500);

	cli_init();

	imu_init();

	while(1){
		imu_data.calib_status=bno055_check_calibration_status(); //this line alone lets the i2c bus crash
		bno055_get_heading(&(imu_data.d_euler_data_h), &(imu_data.d_euler_data_p), &(imu_data.d_euler_data_r)); //this line alone lets the i2c bus crash
//		bno055_get_accel(&(imu_data.accel_x), &(imu_data.accel_y), &(imu_data.accel_z)); //commenting out this line alone still lets the i2c bus be blocked


#ifdef MAVLINK_ON_UART0_ENABLED
		comm_set_tx_flag(CHANNEL_APP_UART, MAV_COMP_ID_IMU);
		comm_mavlink_broadcast(imu_pack_mavlink_raw_int());
#endif

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

		Task_sleep(500);

	}



}
