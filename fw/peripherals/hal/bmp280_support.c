 /*
****************************************************************************
* Copyright (C) 2015 - 2016 Bosch Sensortec GmbH
*
* bmp280_support.c
* Date: 2016/07/01
* Revision: 1.0.6
*
* Usage: Sensor Driver support file for BMP280 sensor
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
/*---------------------------------------------------------------------------*/
/* Includes*/
/*---------------------------------------------------------------------------*/
#include "bmp280.h"
#include "../../../Board.h"
#include "i2c_helper.h"
#include "bmp280_support.h"


/*----------------------------------------------------------------------------*
 *  struct bmp280_t parameters can be accessed by using bmp280
 *	bmp280_t having the following parameters
 *	Bus write function pointer: BMP280_WR_FUNC_PTR
 *	Bus read function pointer: BMP280_RD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *	I2C address: dev_addr
 *	Chip id of the sensor: chip_id
 *---------------------------------------------------------------------------*/
struct bmp280_t bmp280;


s32 bmp280_init(u16 update_period_ms)
{
	/* result of communication results*/
	s32 com_rslt = ERROR;
	/*********************** START INITIALIZATION ************************/
	/*	Based on the user need configure I2C or SPI interface.
	*	It is example code to explain how to use the bma2x2 API*/
	BMP280_I2C_routine();
	/*SPI_routine(); */
	/*--------------------------------------------------------------------------*
	*  This function used to assign the value/reference of
	*	the following parameters
	*	I2C address
	*	Bus Write
	*	Bus read
	*	Chip id
	*-------------------------------------------------------------------------*/
	com_rslt = bmp280_start(&bmp280);

	/*	For initialization it is required to set the mode of
	 *	the sensor as "NORMAL"
	 *	data acquisition/read/write is possible in this mode
	 *	by using the below API able to set the power mode as NORMAL*/
	/* Set the power mode as NORMAL*/
	com_rslt += bmp280_set_power_mode(BMP280_NORMAL_MODE);
	Task_sleep(10); //some delay is apparently needed after mode changes
	/*	For reading the pressure and temperature data it is required to
	 *	set the work mode
	 *	The measurement period in the Normal mode is depends on the setting of
	 *	over sampling setting of pressure, temperature and standby time
	 *
	 *	OSS				pressure OSS	temperature OSS
	 *	ultra low power			x1			x1
	 *	low power			x2			x1
	 *	standard resolution		x4			x1
	 *	high resolution			x8			x2
	 *	ultra high resolution		x16			x2
	 */
	/* The oversampling settings are set by using the following API*/
	com_rslt += bmp280_set_work_mode(BMP280_ULTRA_LOW_POWER_MODE);
	Task_sleep(10); //some delay is apparently needed after mode changes

	/*------------------------------------------------------------------------*
	************************* START GET and SET FUNCTIONS DATA ****************
	*---------------------------------------------------------------------------*/
	/* This API used to Write the standby time of the sensor input
	*	value have to be given*/
	/*	Normal mode comprises an automated perpetual cycling between an (active)
	*	Measurement period and an (inactive) standby period.
	*	The standby time is determined by the contents of the register t_sb.
	*	Standby time can be set using BMP280_STANDBYTIME_125_MS.
	*	Usage Hint : BMP280_set_standbydur(BMP280_STANDBYTIME_125_MS)*/
	if(update_period_ms >= 4000)
		com_rslt += bmp280_set_standby_durn(BMP280_STANDBY_TIME_4000_MS);
	else if(update_period_ms >= 1000)
		com_rslt += bmp280_set_standby_durn(BMP280_STANDBY_TIME_1000_MS);
	else if(update_period_ms >= 250)
		com_rslt += bmp280_set_standby_durn(BMP280_STANDBY_TIME_250_MS);
	else if(update_period_ms >= 63)
		com_rslt += bmp280_set_standby_durn(BMP280_STANDBY_TIME_63_MS);
	else
		com_rslt += bmp280_set_standby_durn(BMP280_STANDBY_TIME_1_MS);
	Task_sleep(10); //some delay is apparently needed after mode changes

//		/* This API used to read back the written value of standby time*/
//		/* The variable used to assign the standby time*/
//		u8 v_standby_time_u8 = BMP280_INIT_VALUE;
//		com_rslt += bmp280_get_standby_durn(&v_standby_time_u8);
	/*-----------------------------------------------------------------*
	************************* END GET and SET FUNCTIONS ****************
	*------------------------------------------------------------------*/

	/************************* END INITIALIZATION *************************/



	return com_rslt;
}


/* This function is an example for reading sensor data
 *	\param: None
 *	\return: communication result
 */
s32 bmp280_data_readout(s32* temp_s32, u32* press_u32)
{
	/* The variables used in individual data read APIs*/
	/* The variable used to read uncompensated temperature*/
	s32 v_data_uncomp_tem_s32 = BMP280_INIT_VALUE;
	/* The variable used to read uncompensated pressure*/
	s32 v_data_uncomp_pres_s32 = BMP280_INIT_VALUE;

	/* result of communication results*/
	s32 com_rslt = ERROR;
	/*------------------------------------------------------------------*
****** INDIVIDUAL APIs TO READ UNCOMPENSATED PRESSURE AND TEMPERATURE*******
*---------------------------------------------------------------------*/
//	/* API is used to read the uncompensated temperature*/
//	com_rslt = bmp280_read_uncomp_temperature(&v_data_uncomp_tem_s32);
//
//	/* API is used to read the uncompensated pressure*/
//	com_rslt += bmp280_read_uncomp_pressure(&v_data_uncomp_pres_s32);
//
//	/* API is used to read the true temperature*/
//	/* Input value as uncompensated temperature*/
//	*temp_s32 = bmp280_compensate_temperature_int32(v_data_uncomp_tem_s32);
//
//	/* API is used to read the true pressure*/
//	/* Input value as uncompensated pressure*/
//	*press_u32 = bmp280_compensate_pressure_int32(v_data_uncomp_pres_s32);


	/* API is used to read the true temperature and pressure*/
	com_rslt += bmp280_read_pressure_temperature(press_u32, temp_s32);

   return com_rslt;
}

#ifdef BMP280_API
/*--------------------------------------------------------------------------*
*	The following function is used to map the I2C bus read, write, delay and
*	device address with global structure bmp280_t
*-------------------------------------------------------------------------*/
s8 BMP280_I2C_routine(void) {
/*--------------------------------------------------------------------------*
 *  By using bmp280 the following structure parameter can be accessed
 *	Bus write function pointer: BMP280_WR_FUNC_PTR
 *	Bus read function pointer: BMP280_RD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *	I2C address: dev_addr
 *--------------------------------------------------------------------------*/
	bmp280.bus_write = BMP280_I2C_bus_write;
	bmp280.bus_read = BMP280_I2C_bus_read;
	bmp280.dev_addr = Board_BMP280_I2CADDR;
	bmp280.delay_msec = BMP280_delay_msek;

	return BMP280_INIT_VALUE;
}


/************** I2C/SPI buffer length ******/

#define	I2C_BUFFER_LEN 		8
#define BUFFER_LENGTH		0xFF
#define BMP280_DATA_INDEX	1
#define BMP280_ADDRESS_INDEX	2

/*-------------------------------------------------------------------*
*	This is a sample code for read and write the data by using I2C/SPI
*	Use either I2C or SPI based on your need
*	The device address defined in the bmp180.c
*
*-----------------------------------------------------------------------*/
 /*	\Brief: The function is used as I2C bus write
 *	\Return : Status of the I2C write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, where data is to be written
 *	\param reg_data : It is a value held in the array,
 *		which is written in the register
 *	\param cnt : The no of bytes of data to be written
 */
s8  BMP280_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	I2C_Transaction i2cTransaction;

	s32 iError = BMP280_INIT_VALUE;
	u8 writebuffer[I2C_BUFFER_LEN] = {BMP280_INIT_VALUE};
	u8 stringpos = BMP280_INIT_VALUE;
	writebuffer[BMP280_INIT_VALUE] = reg_addr;

	for (stringpos = BMP280_INIT_VALUE; stringpos < cnt; stringpos++) {
		writebuffer[stringpos + 1] = *(reg_data + stringpos);
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

	return (s8)iError;
}

 /*	\Brief: The function is used as I2C bus read
 *	\Return : Status of the I2C read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, where data is going to be read
 *	\param reg_data : This is the data read from the sensor, which is held in an array
 *	\param cnt : The no of data to be read
 */
s8  BMP280_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	/* Please take the below function as your reference
	 * to read the data using I2C communication
	 * add your I2C rad function here.
	 * "IERROR = I2C_WRITE_READ_STRING(DEV_ADDR, ARRAY, ARRAY, 1, CNT)"
	 * iError is an return value of SPI write function
	 * Please select your valid return value
	 * In the driver SUCCESS defined as BMP280_INIT_VALUE
	 * and FAILURE defined as -1
	 */

	I2C_Transaction i2cTransaction;

	s32 iError = BMP280_INIT_VALUE;
	u8 readBuffer[cnt];
	u8 stringpos = BMP280_INIT_VALUE;

	i2cTransaction.writeBuf = &reg_addr;
	i2cTransaction.writeCount = sizeof(reg_addr);

	i2cTransaction.readBuf = &readBuffer;
	i2cTransaction.readCount = sizeof(readBuffer);

	i2cTransaction.slaveAddress = dev_addr;

	int ret = I2C_transfer(i2c_helper_handle, &i2cTransaction);

	if (!ret) {
		serial_printf(cli_stdout, "bmp280 read error \n", 0);
		iError = ERROR;
	}else{
		iError = SUCCESS;
	}

	for (stringpos = BMP280_INIT_VALUE; stringpos < cnt; stringpos++) {
		*(reg_data + stringpos) = readBuffer[stringpos];
	}

	return (s8)iError;

}


/*	Brief : The delay routine
 *	\param : delay in ms
*/
void  BMP280_delay_msek(u32 msek)
{
	Task_sleep(msek);
}
#endif
