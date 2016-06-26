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
#include "../../../Board.h"
#include "i2c_helper.h"
#include "SHT2xi2c.h"


char SHT2x_I2C_write(unsigned char dev_addr, unsigned char *reg_data, unsigned char cnt){
	I2C_Transaction i2cTransaction;

        int i = 0;
        unsigned char writebuffer[cnt];

        for (i=0;i<cnt;i++){
            writebuffer[i]=reg_data[i];
        }

	i2cTransaction.readBuf = NULL;
	i2cTransaction.readCount = 0;
	i2cTransaction.writeBuf = writebuffer;
	i2cTransaction.writeCount = cnt;

	i2cTransaction.slaveAddress = dev_addr; //SHT2x_ADR
	int ret = I2C_transfer(i2c_helper_handle, &i2cTransaction);

	if (!ret) {
//		serial_printf(cli_stdout, "SHT2x i2c bus write error\n", 0);
	}

	return ret;
}

char SHT2x_I2C_read(unsigned char dev_addr, unsigned char *reg_data, unsigned char cnt)
{
	I2C_Transaction i2cTransaction;

	i2cTransaction.slaveAddress = dev_addr;
	i2cTransaction.writeBuf = NULL;
	i2cTransaction.writeCount = 0;
	i2cTransaction.readBuf = reg_data;
	i2cTransaction.readCount = cnt;


	int ret = I2C_transfer(i2c_helper_handle, &i2cTransaction);

	if (!ret) {
//		serial_printf(cli_stdout, "SHT2x read error \n", 0);
	}

	return ret;
}

void SHT2x_delay_msek(int msek)
{
	Task_sleep(msek);
}


