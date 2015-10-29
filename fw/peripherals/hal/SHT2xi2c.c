//==============================================================================
//    S E N S I R I O N   AG,  Laubisruetistr. 50, CH-8712 Staefa, Switzerland
//==============================================================================
// Project   :  SHT2x Sample Code (V1.2)
// File      :  I2C_HAL.c
// Author    :  MST
// Controller:  NEC V850/SG3 (uPD70F3740)
// Compiler  :  IAR compiler for V850 (3.50A)
// Brief     :  I2C Hardware abstraction layer
//==============================================================================

//---------- Includes ----------------------------------------------------------
#include "SHT2xi2c.h"
#include "../../../Board.h"
#include "i2c_helper.h"

s8 SHT2x_I2C_write(u8 dev_addr, u8 *reg_data, u8 cnt){
	I2C_Transaction i2cTransaction;

	s8 iError = 1;

	i2cTransaction.slaveAddress = dev_addr; //SHT2x_ADR
	i2cTransaction.readBuf = NULL;
	i2cTransaction.readCount = 0;
	i2cTransaction.writeBuf = reg_data;
	i2cTransaction.writeCount = cnt + 1;

	int ret = I2C_transfer(i2c_helper_handle, &i2cTransaction);

	if (!ret) {
		cli_printf("SHT2x i2c bus write error\n", 0);
		iError = SHT2x_I2C_ERROR;
	}

	return iError;
}

s8 SHT2x_I2C_read(u8 dev_addr, u8 *reg_data, u8 cnt)
{
	I2C_Transaction i2cTransaction;

	u8 iError = 1;

	i2cTransaction.slaveAddress = dev_addr;
	i2cTransaction.writeBuf = NULL;
	i2cTransaction.writeCount = 0;
	i2cTransaction.readBuf = reg_data;
	i2cTransaction.readCount = cnt;


	int ret = I2C_transfer(i2c_helper_handle, &i2cTransaction);

	if (!ret) {
		cli_printf("SHT2x read error \n", 0);
		iError = SHT2x_I2C_ERROR;
	}

	return (s8)iError;
}

void SHT2x_delay_msek(u16 msek)
{
	Task_sleep(msek);
}


