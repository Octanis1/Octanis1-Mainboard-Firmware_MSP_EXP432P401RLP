 /*
****************************************************************************
* Copyright (C) 2015 - 2016 Bosch Sensortec GmbH
*
* si1133_support.c
* Date: 2016/07/01
* Revision: 1.0.6
*
* Usage: Sensor Driver support file for SI1133 sensor
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
#include "SI1133.h"
#include "../../../Board.h"
#include "i2c_helper.h"
#include "SI1133_support.h"


#ifdef SI1133_API

/************** I2C/SPI buffer length ******/
#define	I2C_BUFFER_LEN 		8
#define BUFFER_LENGTH		0xFF

/*-------------------------------------------------------------------*
*	This is a sample code for read and write the data by using I2C/SPI
*	Use either I2C or SPI based on your need
*	The device address defined in the si1133.c
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
s8  SI1133_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	I2C_Transaction i2cTransaction;

	s32 iError = 0;
	u8 writebuffer[I2C_BUFFER_LEN] = {0};
	u8 stringpos = 0;
	writebuffer[0] = reg_addr;

	for (stringpos = 0; stringpos < cnt; stringpos++) {
		writebuffer[stringpos + 1] = *(reg_data + stringpos);
	}

	i2cTransaction.readBuf = NULL;
	i2cTransaction.readCount = 0;
	i2cTransaction.writeBuf = writebuffer;
	i2cTransaction.writeCount = cnt + 1;

	i2cTransaction.slaveAddress = dev_addr; //Board_BNO055_MAINBOARD_I2CADDR;
	int ret = I2C_transfer(i2c_helper_handle, &i2cTransaction);

	if (!ret) {
//		serial_printf(cli_stdout, "si1133 i2c bus write error\n", 0);
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
s8  SI1133_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	/* Please take the below function as your reference
	 * to read the data using I2C communication
	 * add your I2C rad function here.
	 * "IERROR = I2C_WRITE_READ_STRING(DEV_ADDR, ARRAY, ARRAY, 1, CNT)"
	 * iError is an return value of SPI write function
	 * Please select your valid return value
	 * In the driver SUCCESS defined as SI1133_INIT_VALUE
	 * and FAILURE defined as -1
	 */

	I2C_Transaction i2cTransaction;

	s32 iError = 0;
	u8 readBuffer[cnt];
	u8 stringpos = 0;

	i2cTransaction.writeBuf = &reg_addr;
	i2cTransaction.writeCount = sizeof(reg_addr);

	i2cTransaction.readBuf = readBuffer;
	i2cTransaction.readCount = cnt;

	i2cTransaction.slaveAddress = dev_addr;

	int ret = I2C_transfer(i2c_helper_handle, &i2cTransaction);

	if (!ret) {
		serial_printf(cli_stdout, "si1133 read error \n", 0);
		iError = ERROR;
	}else{
		iError = SUCCESS;
	}

	for (stringpos = 0; stringpos < cnt; stringpos++) {
		*(reg_data + stringpos) = readBuffer[stringpos];
	}

	return (s8)iError;

}


/*	Brief : The delay routine
 *	\param : delay in ms
*/
void  SI1133_delay_msek(u32 msek)
{
	Task_sleep(msek);
}
#endif
