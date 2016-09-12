/*
****************************************************************************
* Copyright (C) 2014 Bosch Sensortec GmbH
*
* bno055_support.c
* Date: 2014/12/12
* Revision: 1.0.3 $
*
* Usage: Sensor Driver support file for BNO055 sensor
*
****************************************************************************
* License:
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*   Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
*
*   Redistributions in binary form must reproduce the above copyright
*   notice, this list of conditions and the following disclaimer in the
*   documentation and/or other materials provided with the distribution.
*
*   Neither the name of the copyright holder nor the names of the
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
* OR CONTRIBUTORS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
* OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*
* The information provided is believed to be accurate and reliable.
* The copyright holder assumes no responsibility
* for the consequences of use
* of such information nor for any infringement of patents or
* other rights of third parties which may result from its use.
* No license is granted by implication or otherwise under any patent or
* patent rights of the copyright holder.
**************************************************************************/

#include "../../../Board.h"
#include "i2c_helper.h"
#include "bno055_support.h"

/*----------------------------------------------------------------------------*
 *  struct bno055_t parameters can be accessed by using BNO055
 *	BNO055_t having the following parameters
 *	Bus write function pointer: BNO055_WR_FUNC_PTR
 *	Bus read function pointer: BNO055_RD_FUNC_PTR
 *	Burst read function pointer: BNO055_BRD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *	I2C address: dev_addr
 *	Chip id of the sensor: chip_id
*---------------------------------------------------------------------------*/
struct bno055_t my_bno055;
/* This function is an example for reading sensor data
 *	\param: None
 *	\return: communication result
 */

s32 imu_init() //function to be called before starting the imu task.
{
	//TODO: verify correct axis alignment!

	/* Variable used to return value of
	communication routine*/
	s32 comres = ERROR;
	/*---------------------------------------------------------------------------*
	 *********************** START INITIALIZATION ************************
	 *--------------------------------------------------------------------------*/
	/*	Based on the user need configure I2C interface.
	 *	It is example code to explain how to use the bno055 API*/
		BNO055_I2C_routine();
	/*--------------------------------------------------------------------------*
	 *  This function used to assign the value/reference of
	 *	the following parameters
	 *	I2C address
	 *	Bus Write
	 *	Bus read
	 *	Chip id
	 *	Page id
	 *	Accel revision id
	 *	Mag revision id
	 *	Gyro revision id
	 *	Boot loader revision id
	 *	Software revision id
	 *-------------------------------------------------------------------------*/
		comres = bno055_init(&my_bno055);

	/*	For initializing the BNO sensor it is required to the operation mode
		of the sensor as NORMAL
		Normal mode can set from the register
		Page - page0
		register - 0x3E
		bit positions - 0 and 1*/
		/* set the power mode as NORMAL*/
		comres += bno055_set_power_mode(POWER_MODE_NORMAL);
	/*--------------------------------------------------------------------------*
	************************* END INITIALIZATION *************************
	*---------------------------------------------------------------------------*/
	return comres;
}



s8 bno055_get_heading(double *d_euler_data_h, double *d_euler_data_p, double *d_euler_data_r)
{
	/* Variable used to return value of
	communication routine*/
	s32 comres = ERROR;
	/************************* START READ RAW FUSION DATA ********
	For reading fusion data it is required to set the
	operation modes of the sensor
	operation mode can set from the register
	page - page0
	register - 0x3D
	bit - 0 to 3
	for sensor data read following operation mode have to set
	*FUSION MODE
		*0x08 - OPERATION_MODE_IMUPLUS
		*0x09 - OPERATION_MODE_COMPASS
		*0x0A - OPERATION_MODE_M4G
		*0x0B - OPERATION_MODE_NDOF_FMC_OFF
		*0x0C - OPERATION_MODE_NDOF
		based on the user need configure the operation mode*/
	comres = bno055_set_operation_mode(OPERATION_MODE_NDOF);
	/*	API used to read Euler data output as double  - degree and radians
		float functions also available in the BNO055 API */
	comres += bno055_convert_double_euler_h_deg(d_euler_data_h);
	comres += bno055_convert_double_euler_r_deg(d_euler_data_r);
	comres += bno055_convert_double_euler_p_deg(d_euler_data_p);

	return comres;
}

s8 bno055_get_accel(int16_t *acc_x, int16_t *acc_y, int16_t *acc_z)
{
	/* Variable used to return value of
	communication routine*/
	s8 comres = ERROR;
	/*	For reading sensor raw data it is required to set the
		operation modes of the sensor
		operation mode can set from the register
		page - page0
		register - 0x3D
		bit - 0 to 3
		for sensor data read following operation mode have to set
		 * SENSOR MODE
			*0x01 - OPERATION_MODE_ACCONLY
			*0x02 - OPERATION_MODE_MAGONLY
			*0x03 - OPERATION_MODE_GYRONLY
			*0x04 - OPERATION_MODE_ACCMAG
			*0x05 - OPERATION_MODE_ACCGYRO
			*0x06 - OPERATION_MODE_MAGGYRO
			*0x07 - OPERATION_MODE_AMG
			based on the user need configure the operation mode*/
//		comres += bno055_set_operation_mode(OPERATION_MODE_AMG);

	/*	Raw Linear accel X, Y and Z data can read from the register
		page - page 0
		register - 0x28 to 0x2D */
//		comres += bno055_read_linear_accel_x(acc_x);
//		comres += bno055_read_linear_accel_y(acc_y);
//		comres += bno055_read_linear_accel_z(acc_z);
		comres += bno055_read_accel_x(acc_x);
		comres += bno055_read_accel_y(acc_y);
		comres += bno055_read_accel_z(acc_z);

	return comres;
}


/*!
 *	@brief This API used to read	calibration status
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
unsigned char bno055_check_calibration_status()
{
	//from the datasheet:
	/*Therefore, it is highly recommended to check the magnetometer calibration status periodically.
	If the value of the two bits ‘MAG Calib Status’ is 3, then it means that the magnetometer is fully
	calibrated and ready to go. If the value is 2, then the sensor fusion’s performance is still OK. If
	the value is 1, then it is highly recommended to perform a Figure-8 motion to calibrate the
	magnetometer. And if the value is 0, this means that the magnetometer just got disturbed by
	the magnetic interference fields nearby or the environment’s magnetic fields have just
	changed. And therefore the magnetometer calibration must be performed. For further details
	please refer section ‘3.10 Calibration’ in the datasheet.*/

	unsigned char accel_calib_status = 0;
	unsigned char gyro_calib_status = 0;
	unsigned char mag_calib_status = 0;
	unsigned char sys_calib_status = 0;
	bno055_get_accel_calib_stat(&accel_calib_status);
	bno055_get_mag_calib_stat(&mag_calib_status);
	bno055_get_gyro_calib_stat(&gyro_calib_status);
	bno055_get_sys_calib_stat(&sys_calib_status);

	return accel_calib_status+gyro_calib_status+mag_calib_status+sys_calib_status;
}




//s32 bno055_data_readout_template(void)
//{
//	/* Variable used to return value of
//	communication routine*/
//	s32 comres = ERROR;
//	/* variable used to set the power mode of the sensor*/
//	u8 power_mode = BNO055_ZERO_U8X;
//	/*********read raw accel data***********/
//	/* variable used to read the accel x data */
//	s16 accel_datax = BNO055_ZERO_U8X;
//	 /* variable used to read the accel y data */
//	s16 accel_datay = BNO055_ZERO_U8X;
//	/* variable used to read the accel z data */
//	s16 accel_dataz = BNO055_ZERO_U8X;
//	/* variable used to read the accel xyz data */
//	struct bno055_accel_t accel_xyz;
//
//	/*********read raw mag data***********/
//	/* variable used to read the mag x data */
//	s16 mag_datax  = BNO055_ZERO_U8X;
//	/* variable used to read the mag y data */
//	s16 mag_datay  = BNO055_ZERO_U8X;
//	/* variable used to read the mag z data */
//	s16 mag_dataz  = BNO055_ZERO_U8X;
//	/* structure used to read the mag xyz data */
//	struct bno055_mag_t mag_xyz;
//
//	/***********read raw gyro data***********/
//	/* variable used to read the gyro x data */
//	s16 gyro_datax = BNO055_ZERO_U8X;
//	/* variable used to read the gyro y data */
//	s16 gyro_datay = BNO055_ZERO_U8X;
//	 /* variable used to read the gyro z data */
//	s16 gyro_dataz = BNO055_ZERO_U8X;
//	 /* structure used to read the gyro xyz data */
//	struct bno055_gyro_t gyro_xyz;
//
//	/*************read raw Euler data************/
//	/* variable used to read the euler h data */
//	s16 euler_data_h = BNO055_ZERO_U8X;
//	 /* variable used to read the euler r data */
//	s16 euler_data_r = BNO055_ZERO_U8X;
//	/* variable used to read the euler p data */
//	s16 euler_data_p = BNO055_ZERO_U8X;
//	/* structure used to read the euler hrp data */
//	struct bno055_euler_t euler_hrp;
//
//	/************read raw quaternion data**************/
//	/* variable used to read the quaternion w data */
//	s16 quaternion_data_w = BNO055_ZERO_U8X;
//	/* variable used to read the quaternion x data */
//	s16 quaternion_data_x = BNO055_ZERO_U8X;
//	/* variable used to read the quaternion y data */
//	s16 quaternion_data_y = BNO055_ZERO_U8X;
//	/* variable used to read the quaternion z data */
//	s16 quaternion_data_z = BNO055_ZERO_U8X;
//	/* structure used to read the quaternion wxyz data */
//	struct bno055_quaternion_t quaternion_wxyz;
//
//	/************read raw linear acceleration data***********/
//	/* variable used to read the linear accel x data */
//	s16 linear_accel_data_x = BNO055_ZERO_U8X;
//	/* variable used to read the linear accel y data */
//	s16 linear_accel_data_y = BNO055_ZERO_U8X;
//	/* variable used to read the linear accel z data */
//	s16 linear_accel_data_z = BNO055_ZERO_U8X;
//	/* structure used to read the linear accel xyz data */
//	struct bno055_linear_accel_t linear_acce_xyz;
//
//	/*****************read raw gravity sensor data****************/
//	/* variable used to read the gravity x data */
//	s16 gravity_data_x = BNO055_ZERO_U8X;
//	/* variable used to read the gravity y data */
//	s16 gravity_data_y = BNO055_ZERO_U8X;
//	/* variable used to read the gravity z data */
//	s16 gravity_data_z = BNO055_ZERO_U8X;
//	/* structure used to read the gravity xyz data */
//	struct bno055_gravity_t gravity_xyz;
//
//	/*************read accel converted data***************/
//	/* variable used to read the accel x data output as m/s2 or mg */
//	double d_accel_datax = BNO055_ZERO_U8X;
//	/* variable used to read the accel y data output as m/s2 or mg */
//	double d_accel_datay = BNO055_ZERO_U8X;
//	/* variable used to read the accel z data output as m/s2 or mg */
//	double d_accel_dataz = BNO055_ZERO_U8X;
//	/* structure used to read the accel xyz data output as m/s2 or mg */
//	struct bno055_accel_double_t d_accel_xyz;
//
//	/******************read mag converted data********************/
//	/* variable used to read the mag x data output as uT*/
//	double d_mag_datax = BNO055_ZERO_U8X;
//	/* variable used to read the mag y data output as uT*/
//	double d_mag_datay = BNO055_ZERO_U8X;
//	/* variable used to read the mag z data output as uT*/
//	double d_mag_dataz = BNO055_ZERO_U8X;
//	/* structure used to read the mag xyz data output as uT*/
//	struct bno055_mag_double_t d_mag_xyz;
//
//	/*****************read gyro converted data************************/
//	/* variable used to read the gyro x data output as dps or rps */
//	double d_gyro_datax = BNO055_ZERO_U8X;
//	/* variable used to read the gyro y data output as dps or rps */
//	double d_gyro_datay = BNO055_ZERO_U8X;
//	/* variable used to read the gyro z data output as dps or rps */
//	double d_gyro_dataz = BNO055_ZERO_U8X;
//	/* structure used to read the gyro xyz data output as dps or rps */
//	struct bno055_gyro_double_t d_gyro_xyz;
//
//	/*******************read euler converted data*******************/
//	/* variable used to read the euler h data output as degree or radians */
//	double d_euler_data_h = BNO055_ZERO_U8X;
//	/* variable used to read the euler r data output as degree or radians */
//	double d_euler_data_r = BNO055_ZERO_U8X;
//	/* variable used to read the euler p data output as degree or radians */
//	double d_euler_data_p = BNO055_ZERO_U8X;
//	/* structure used to read the euler hrp data output as as degree or radians */
//	struct bno055_euler_double_t d_euler_hpr;
//
//	/*********************read linear acceleration converted data*************************/
//	/* variable used to read the linear accel x data output as m/s2*/
//	double d_linear_accel_datax = BNO055_ZERO_U8X;
//	/* variable used to read the linear accel y data output as m/s2*/
//	double d_linear_accel_datay = BNO055_ZERO_U8X;
//	/* variable used to read the linear accel z data output as m/s2*/
//	double d_linear_accel_dataz = BNO055_ZERO_U8X;
//	/* structure used to read the linear accel xyz data output as m/s2*/
//	struct bno055_linear_accel_double_t d_linear_accel_xyz;
//
//	/********************Gravity converted data*****************************/
//	/* variable used to read the gravity sensor x data output as m/s2*/
//	double d_gravity_data_x = BNO055_ZERO_U8X;
//	/* variable used to read the gravity sensor y data output as m/s2*/
//	double d_gravity_data_y = BNO055_ZERO_U8X;
//	/* variable used to read the gravity sensor z data output as m/s2*/
//	double d_gravity_data_z = BNO055_ZERO_U8X;
//	/* structure used to read the gravity xyz data output as m/s2*/
//	struct bno055_gravity_double_t d_gravity_xyz;
//
//
///************************* START READ RAW SENSOR DATA****************/
//
///*	Using BNO055 sensor we can read the following sensor data and
//	virtual sensor data
//	Sensor data:
//		Accel
//		Mag
//		Gyro
//	Virtual sensor data
//		Euler
//		Quaternion
//		Linear acceleration
//		Gravity sensor */
///*	For reading sensor raw data it is required to set the
//	operation modes of the sensor
//	operation mode can set from the register
//	page - page0
//	register - 0x3D
//	bit - 0 to 3
//	for sensor data read following operation mode have to set
//	 * SENSOR MODE
//		*0x01 - OPERATION_MODE_ACCONLY
//		*0x02 - OPERATION_MODE_MAGONLY
//		*0x03 - OPERATION_MODE_GYRONLY
//		*0x04 - OPERATION_MODE_ACCMAG
//		*0x05 - OPERATION_MODE_ACCGYRO
//		*0x06 - OPERATION_MODE_MAGGYRO
//		*0x07 - OPERATION_MODE_AMG
//		based on the user need configure the operation mode*/
//	comres += bno055_set_operation_mode(OPERATION_MODE_AMG);
///*	Raw accel X, Y and Z data can read from the register
//	page - page 0
//	register - 0x08 to 0x0D*/
//	comres += bno055_read_accel_x(&accel_datax);
//	comres += bno055_read_accel_y(&accel_datay);
//	comres += bno055_read_accel_z(&accel_dataz);
//	comres += bno055_read_accel_xyz(&accel_xyz);
///*	Raw mag X, Y and Z data can read from the register
//	page - page 0
//	register - 0x0E to 0x13*/
//	comres += bno055_read_mag_x(&mag_datax);
//	comres += bno055_read_mag_y(&mag_datay);
//	comres += bno055_read_mag_z(&mag_dataz);
//	comres += bno055_read_mag_xyz(&mag_xyz);
///*	Raw gyro X, Y and Z data can read from the register
//	page - page 0
//	register - 0x14 to 0x19*/
//	comres += bno055_read_gyro_x(&gyro_datax);
//	comres += bno055_read_gyro_y(&gyro_datay);
//	comres += bno055_read_gyro_z(&gyro_dataz);
//	comres += bno055_read_gyro_xyz(&gyro_xyz);
//
///************************* END READ RAW SENSOR DATA****************/
//
///************************* START READ RAW FUSION DATA ********
// 	For reading fusion data it is required to set the
//	operation modes of the sensor
//	operation mode can set from the register
//	page - page0
//	register - 0x3D
//	bit - 0 to 3
//	for sensor data read following operation mode have to set
//	*FUSION MODE
//		*0x08 - OPERATION_MODE_IMUPLUS
//		*0x09 - OPERATION_MODE_COMPASS
//		*0x0A - OPERATION_MODE_M4G
//		*0x0B - OPERATION_MODE_NDOF_FMC_OFF
//		*0x0C - OPERATION_MODE_NDOF
//		based on the user need configure the operation mode*/
//	comres += bno055_set_operation_mode(OPERATION_MODE_NDOF);
///*	Raw Euler H, R and P data can read from the register
//	page - page 0
//	register - 0x1A to 0x1E */
//	comres += bno055_read_euler_h(&euler_data_h);
//	comres += bno055_read_euler_r(&euler_data_r);
//	comres += bno055_read_euler_p(&euler_data_p);
//	comres += bno055_read_euler_hrp(&euler_hrp);
///*	Raw Quaternion W, X, Y and Z data can read from the register
//	page - page 0
//	register - 0x20 to 0x27 */
//	comres += bno055_read_quaternion_w(&quaternion_data_w);
//	comres += bno055_read_quaternion_x(&quaternion_data_x);
//	comres += bno055_read_quaternion_y(&quaternion_data_y);
//	comres += bno055_read_quaternion_z(&quaternion_data_z);
//	comres += bno055_read_quaternion_wxyz(&quaternion_wxyz);
///*	Raw Linear accel X, Y and Z data can read from the register
//	page - page 0
//	register - 0x28 to 0x2D */
//	comres += bno055_read_linear_accel_x(&linear_accel_data_x);
//	comres += bno055_read_linear_accel_y(&linear_accel_data_y);
//	comres += bno055_read_linear_accel_z(&linear_accel_data_z);
//	comres += bno055_read_linear_accel_xyz(&linear_acce_xyz);
///*	Raw Gravity sensor X, Y and Z data can read from the register
//	page - page 0
//	register - 0x2E to 0x33 */
//	comres += bno055_read_gravity_x(&gravity_data_x);
//	comres += bno055_read_gravity_y(&gravity_data_y);
//	comres += bno055_read_gravity_z(&gravity_data_z);
//	comres += bno055_read_gravity_xyz(&gravity_xyz);
///************************* END READ RAW FUSION DATA  ************/
//
///******************START READ CONVERTED SENSOR DATA****************/
///*	API used to read accel data output as double  - m/s2 and mg
//	float functions also available in the BNO055 API */
//	comres += bno055_convert_double_accel_x_msq(&d_accel_datax);
//	comres += bno055_convert_double_accel_x_mg(&d_accel_datax);
//	comres += bno055_convert_double_accel_y_msq(&d_accel_datay);
//	comres += bno055_convert_double_accel_y_mg(&d_accel_datay);
//	comres += bno055_convert_double_accel_z_msq(&d_accel_dataz);
//	comres += bno055_convert_double_accel_z_mg(&d_accel_dataz);
//	comres += bno055_convert_double_accel_xyz_msq(&d_accel_xyz);
//	comres += bno055_convert_double_accel_xyz_mg(&d_accel_xyz);
//
///*	API used to read mag data output as double  - uT(micro Tesla)
//	float functions also available in the BNO055 API */
//	comres += bno055_convert_double_mag_x_uT(&d_mag_datax);
//	comres += bno055_convert_double_mag_y_uT(&d_mag_datay);
//	comres += bno055_convert_double_mag_z_uT(&d_mag_dataz);
//	comres += bno055_convert_double_mag_xyz_uT(&d_mag_xyz);
//
///*	API used to read gyro data output as double  - dps and rps
//	float functions also available in the BNO055 API */
//	comres += bno055_convert_double_gyro_x_dps(&d_gyro_datax);
//	comres += bno055_convert_double_gyro_y_dps(&d_gyro_datay);
//	comres += bno055_convert_double_gyro_z_dps(&d_gyro_dataz);
//	comres += bno055_convert_double_gyro_x_rps(&d_gyro_datax);
//	comres += bno055_convert_double_gyro_y_rps(&d_gyro_datay);
//	comres += bno055_convert_double_gyro_z_rps(&d_gyro_dataz);
//	comres += bno055_convert_double_gyro_xyz_dps(&d_gyro_xyz);
//	comres += bno055_convert_double_gyro_xyz_rps(&d_gyro_xyz);
//
///*	API used to read Euler data output as double  - degree and radians
//	float functions also available in the BNO055 API */
//	comres += bno055_convert_double_euler_h_deg(&d_euler_data_h);
//	comres += bno055_convert_double_euler_r_deg(&d_euler_data_r);
//	comres += bno055_convert_double_euler_p_deg(&d_euler_data_p);
//	comres += bno055_convert_double_euler_h_rad(&d_euler_data_h);
//	comres += bno055_convert_double_euler_r_rad(&d_euler_data_r);
//	comres += bno055_convert_double_euler_p_rad(&d_euler_data_p);
//	comres += bno055_convert_double_euler_hpr_deg(&d_euler_hpr);
//	comres += bno055_convert_double_euler_hpr_rad(&d_euler_hpr);
//
///*	API used to read Linear acceleration data output as m/s2
//	float functions also available in the BNO055 API */
//	comres += bno055_convert_double_linear_accel_x_msq(&d_linear_accel_datax);
//	comres += bno055_convert_double_linear_accel_y_msq(&d_linear_accel_datay);
//	comres += bno055_convert_double_linear_accel_z_msq(&d_linear_accel_dataz);
//	comres += bno055_convert_double_linear_accel_xyz_msq(&d_linear_accel_xyz);
//
///*	API used to read Gravity sensor data output as m/s2
//	float functions also available in the BNO055 API */
//	comres += bno055_convert_gravity_double_x_msq(&d_gravity_data_x);
//	comres += bno055_convert_gravity_double_y_msq(&d_gravity_data_y);
//	comres += bno055_convert_gravity_double_z_msq(&d_gravity_data_z);
//	comres += bno055_convert_double_gravity_xyz_msq(&d_gravity_xyz);
///*-----------------------------------------------------------------------*
//************************* START DE-INITIALIZATION ***********************
//*-------------------------------------------------------------------------*/
///*	For de - initializing the BNO sensor it is required to the operation mode
//	of the sensor as SUSPEND
//	Suspend mode can set from the register
//	Page - page0
//	register - 0x3E
//	bit positions - 0 and 1*/
//	power_mode = POWER_MODE_SUSPEND; /* set the power mode as SUSPEND*/
//	comres += bno055_set_power_mode(power_mode);
//
///*---------------------------------------------------------------------*
//************************* END DE-INITIALIZATION **********************
//*---------------------------------------------------------------------*/
//return comres;
//}

/*--------------------------------------------------------------------------*
*	The following function is used to map the I2C bus read, write, delay and
*	device address with global structure bno055_t
*-------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------*
 *  By using bno055 the following structure parameter can be accessed
 *	Bus write function pointer: BNO055_WR_FUNC_PTR
 *	Bus read function pointer: BNO055_RD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *	I2C address: dev_addr
 *--------------------------------------------------------------------------*/
 s8 BNO055_I2C_routine(void) {

	my_bno055.bus_write = BNO055_I2C_bus_write;
	my_bno055.bus_read = BNO055_I2C_bus_read;
	my_bno055.delay_msec = BNO055_delay_msek;
	my_bno055.dev_addr = Board_BNO055_MAINBOARD_I2CADDR; //TODO: integarte this in the API funcition in order to chose between weather strip and mainboard sensor.

	return BNO055_ZERO_U8X;
}

/************** I2C buffer length******/

#define	I2C_BUFFER_LEN 8
#define I2C0 5
/*-------------------------------------------------------------------*
*
*	This is a sample code for read and write the data by using I2C
*	Use either I2C  based on your need
*	The device address defined in the bno055.h file
*
*-----------------------------------------------------------------------*/

/*	\Brief: The function is used as I2C bus write
 *	\Return : Status of the I2C write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be written
 *	\param reg_data : It is a value hold in the array,
 *		will be used for write the value into the register
 *	\param cnt : The no of byte of data to be write
 */
s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	I2C_Transaction i2cTransaction;

	s32 iError = BNO055_ZERO_U8X;
	u8 writebuffer[I2C_BUFFER_LEN];
	u8 stringpos = BNO055_ZERO_U8X;
	writebuffer[BNO055_ZERO_U8X] = reg_addr;
	for (stringpos = BNO055_ZERO_U8X; stringpos < cnt; stringpos++) {
		writebuffer[stringpos + BNO055_ONE_U8X] = *(reg_data + stringpos);
	}

	i2cTransaction.readBuf = NULL;
	i2cTransaction.readCount = 0;
	i2cTransaction.writeBuf = writebuffer;
	i2cTransaction.writeCount = cnt + 1;

	i2cTransaction.slaveAddress = dev_addr; //Board_BNO055_MAINBOARD_I2CADDR;
	int ret = I2C_transfer(i2c_helper_handle, &i2cTransaction);

	if (!ret) {
//		serial_printf(cli_stdout, "bme280 i2c bus write error\n", 0);
		iError = ERROR;
	}

	/*
	* Please take the below function as your reference for
	* write the data using I2C communication
	* "IERROR = I2C_WRITE_STRING(DEV_ADDR, ARRAY, CNT+1)"
	* add your I2C write function here
	* iError is an return value of I2C read function
	* Please select your valid return value
	* In the driver SUCCESS defined as 0
    * and FAILURE defined as -1
	* Note :
	* This is a full duplex operation,
	* The first read data is discarded, for that extra write operation
	* have to be initiated. For that cnt+1 operation done in the I2C write string function
	* For more information please refer data sheet SPI communication:
	*/
	return (s8)iError;
}

 /*	\Brief: The function is used as I2C bus read
 *	\Return : Status of the I2C read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be read
 *	\param reg_data : This data read from the sensor, which is hold in an array
 *	\param cnt : The no of byte of data to be read
 */
s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	I2C_Transaction i2cTransaction;

	s32 iError = BNO055_ZERO_U8X;
	u8 readBuffer[cnt];
	u8 stringpos = BNO055_ZERO_U8X;
	//array[BNO055_ZERO_U8X] = reg_addr;
	/* Please take the below function as your reference
	 * for read the data using I2C communication
	 * add your I2C rad function here.
	 * "IERROR = I2C_WRITE_READ_STRING(DEV_ADDR, ARRAY, ARRAY, 1, CNT)"
	 * iError is an return value of SPI write function
	 * Please select your valid return value
     * In the driver SUCCESS defined as 0
     * and FAILURE defined as -1
	 */
	i2cTransaction.writeBuf = &reg_addr;
	i2cTransaction.writeCount = sizeof(reg_addr);

	i2cTransaction.readBuf = &readBuffer;
	i2cTransaction.readCount = sizeof(readBuffer);

	i2cTransaction.slaveAddress = dev_addr;

	int ret = I2C_transfer(i2c_helper_handle, &i2cTransaction);

	if (!ret) {
		serial_printf(cli_stdout, "bno055 read error \n", 0);
		iError = ERROR;
	}else{
		iError = SUCCESS;
	}


	for (stringpos = BNO055_ZERO_U8X; stringpos < cnt; stringpos++) {
		*(reg_data + stringpos) = readBuffer[stringpos];
	}
	return (s8)iError;
}
/*	Brief : The delay routine
 *	\param : delay in ms
*/
void BNO055_delay_msek(u32 msek)
{
	Task_sleep(msek);
}


