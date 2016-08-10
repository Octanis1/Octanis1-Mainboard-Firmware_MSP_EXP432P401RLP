 /*
****************************************************************************
* Copyright (C) 2014 - 2015 Bosch Sensortec GmbH
*
* bmp280_support.c
* Date: 2015/03/27
* Revision: 1.0.5
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
#include "../../../Board.h"
#include "i2c_helper.h"
#include "bme280i2c.h"

#define HUMIDITY_CONSTANT   	10.24 //value of 42313 represents 42313 / 1024 = 41.321 %rH


/*----------------------------------------------------------------------------*
 *  struct bme280_t parameters can be accessed by using bme280
 *	bme280_t having the following parameters
 *	Bus write function pointer: BME280_WR_FUNC_PTR
 *	Bus read function pointer: BME280_RD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *	I2C address: dev_addr
 *	Chip id of the sensor: chip_id
 *---------------------------------------------------------------------------*/
struct bme280_t bme280;

s32 bme280_init()
{
	/* result of communication results*/
		s32 com_rslt = ERROR;



	 /*********************** START INITIALIZATION ************************/
	  /*	Based on the user need configure I2C or SPI interface.
	  *	It is example code to explain how to use the bme280 API*/
		bme280I2C_routine();

	/*--------------------------------------------------------------------------*
	 *  This function used to assign the value/reference of
	 *	the following parameters
	 *	I2C address
	 *	Bus Write
	 *	Bus read
	 *	Chip id
	*-------------------------------------------------------------------------*/
		com_rslt = bme280_start(&bme280);


		/*	For initialization it is required to set the mode of
		 *	the sensor as "NORMAL"
		 *	data acquisition/read/write is possible in this mode
		 *	by using the below API able to set the power mode as NORMAL*/
		com_rslt += bme280_set_power_mode(BME280_NORMAL_MODE);

		/*	For reading the pressure, humidity and temperature data it is required to
		 *	set the OSS setting of humidity, pressure and temperature
		 * The "BME280_CTRLHUM_REG_OSRSH" register sets the humidity
		 * data acquisition options of the device.
		 * changes to this registers only become effective after a write operation to
		 * "BME280_CTRLMEAS_REG" register.
		 * In the code automated reading and writing of "BME280_CTRLHUM_REG_OSRSH"
		 * register first set the "BME280_CTRLHUM_REG_OSRSH" and then read and write
		 * the "BME280_CTRLMEAS_REG" register in the function*/
		com_rslt += bme280_set_oversamp_humidity(BME280_OVERSAMP_1X);
		/* set the pressure oversampling*/
		com_rslt += bme280_set_oversamp_pressure(BME280_OVERSAMP_1X);
		/* set the temperature oversampling*/
		com_rslt += bme280_set_oversamp_temperature(BME280_OVERSAMP_1X);


		/************************* END INITIALIZATION *************************/
		return com_rslt;
}



s32 bme280_data_readout(int* temp_s32, unsigned int* press_u32, unsigned int* humity_u32)
{
	/* The variable used to read real temperature*/
	(*temp_s32) = BME280_INIT_VALUE;
	/* The variable used to read real pressure*/
	(*press_u32) = BME280_INIT_VALUE;
	/* The variable used to read real humidity*/
	(*humity_u32) = BME280_INIT_VALUE;
	/* result of communication results*/
	s32 com_rslt = ERROR;

/*------------------------------------------------------------------*
************ START READ TRUE PRESSURE, TEMPERATURE
AND HUMIDITY DATA ********
*---------------------------------------------------------------------*/
	/* Set the power mode as FORCED:
	 * When the measurement is finished, the sensor returns to sleep mode and
	 * the measurement results can be obtained from the data registers.
	 * For a next measurement, forced mode needs to be selected again*/
	com_rslt += bme280_set_power_mode(BME280_FORCED_MODE);

	/* API is used to read the true temperature, humidity and pressure*/
	com_rslt += bme280_read_pressure_temperature_humidity(press_u32, temp_s32, humity_u32);

	// convert humidity value:
	(*humity_u32) = (u16)((float)(*humity_u32))/HUMIDITY_CONSTANT; //value of 42313 represents 42313 / 1024 = 41.321 %rH

/*--------------------------------------------------------------------*
************ END READ TRUE PRESSURE, TEMPERATURE AND HUMIDITY ********
*-------------------------------------------------------------------------*/

return com_rslt;
}

#define MASK_DATA1	0xFF
#define MASK_DATA2	0x80
#define MASK_DATA3	0x7F
/*--------------------------------------------------------------------------*
*	The following function is used to map the I2C bus read, write, delay and
*	device address with global structure bme280
*-------------------------------------------------------------------------*/
s8 bme280I2C_routine(void) {
/*--------------------------------------------------------------------------*
 *  By using bme280 the following structure parameter can be accessed
 *	Bus write function pointer: BME280_WR_FUNC_PTR
 *	Bus read function pointer: BME280_RD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *	I2C address: dev_addr
 *--------------------------------------------------------------------------*/
	bme280.bus_write = BME280_I2C_bus_write;
	bme280.bus_read = BME280_I2C_bus_read;
	bme280.dev_addr = Board_BME280_I2CADDR;
	bme280.delay_msec = BME280_delay_msek;

	return BME280_INIT_VALUE;
}



/************** I2C/SPI buffer length ******/
#define	I2C_BUFFER_LEN 8

/*-------------------------------------------------------------------*
*	This is a sample code for read and write the data by using I2C/SPI
*	Use either I2C or SPI based on your need
*	The device address defined in the bme280.h file
*-----------------------------------------------------------------------*/
#define BME280_ONE_U8X 1 //TODO:not sure
 /*	\Brief: The function is used as I2C bus write
 *	\Return : Status of the I2C write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be written
 *	\param reg_data : It is a value hold in the array,
 *		will be used for write the value into the register
 *	\param cnt : The no of byte of data to be write
 */
s8 BME280_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	I2C_Transaction i2cTransaction;

	s32 iError = BME280_INIT_VALUE;
	u8 writebuffer[I2C_BUFFER_LEN];
	u8 stringpos = BME280_INIT_VALUE;
	writebuffer[BME280_INIT_VALUE] = reg_addr;
	for (stringpos = BME280_INIT_VALUE; stringpos < cnt; stringpos++) {
		writebuffer[stringpos+BME280_ONE_U8X] = *(reg_data + stringpos);
	}

	i2cTransaction.readBuf = NULL;
	i2cTransaction.readCount = 0;
	i2cTransaction.writeBuf = writebuffer;
	i2cTransaction.writeCount = cnt + 1;

	i2cTransaction.slaveAddress = dev_addr; //Board_BME280_I2CADDR;
	int ret = I2C_transfer(i2c_helper_handle, &i2cTransaction);

	if (!ret) {
		//serial_printf(cli_stdout, "bme280 i2c bus write error\n", 0);
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
 *	\param cnt : The no of data byte of to be read
 */
s8 BME280_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	I2C_Transaction i2cTransaction;

	s32 iError = BME280_INIT_VALUE;
	u8 readBuffer[cnt];
	u8 stringpos = BME280_INIT_VALUE;
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
	//	serial_printf(cli_stdout, "bme280 read error \n", 0);
		iError = ERROR;
	}else{
		iError = SUCCESS;
	}


	for (stringpos = BME280_INIT_VALUE; stringpos < cnt; stringpos++) {
		*(reg_data + stringpos) = readBuffer[stringpos];
	}
	return (s8)iError;
}


/*	Brief : The delay routine
 *	\param : delay in ms
*/
void BME280_delay_msek(u32 msek)
{
	Task_sleep(msek);
}



