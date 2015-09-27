/*
 * i2c_helper.c
 *
 *  Created on: 19 Aug 2015
 *      Author: Sam
 */
#include "../../../Board.h"
#include "i2c_helper.h"

I2C_Handle i2c_helper_handle;

I2C_Handle i2c_helper_get_handle(){

	/* Initialise I2C Bus */
	I2C_Params      params;
	I2C_Params_init(&params);
	i2c_helper_handle = I2C_open(Board_I2C0, &params);

	if (!i2c_helper_handle) {
		cli_printf("I2C did not or already open\n", 0);
	}

	return i2c_helper_handle;
}

void i2c_helper_init_handle(){

	/* Initialise I2C Bus */
	I2C_Params      params;
	I2C_Params_init(&params);
	i2c_helper_handle = I2C_open(Board_I2C0, &params);

	if (!i2c_helper_handle) {
		cli_printf("I2C did not or already open\n", 0);
	}
}


uint8_t read8(unsigned char i2c_addr, I2C_Handle handle, uint8_t register_addr){

	uint8_t readBuffer; //16 bits to be read only
	uint16_t returnValue = 0;
	I2C_Transaction i2cTransaction;


	if(handle != NULL){

		i2cTransaction.writeBuf = &register_addr;
		i2cTransaction.writeCount = sizeof(register_addr);

		i2cTransaction.readBuf = &readBuffer;
		i2cTransaction.readCount = sizeof(readBuffer); //make sure we read only 2 bytes

		i2cTransaction.slaveAddress = i2c_addr;

		int ret = I2C_transfer(handle, &i2cTransaction);

		if (!ret) {
		    cli_printf("read8 error \n", 0);
		}else{
			returnValue = readBuffer;
		}

	}

	return returnValue;
}

uint16_t read16(unsigned char i2c_addr, I2C_Handle handle, uint8_t register_addr){

	uint8_t readBuffer[2]; //16 bits to be read only
	uint16_t returnValue = 0;
	I2C_Transaction i2cTransaction;

	if(handle != NULL){

		i2cTransaction.writeBuf = &register_addr;
		i2cTransaction.writeCount = sizeof(register_addr);

		i2cTransaction.readBuf = readBuffer;
		i2cTransaction.readCount = 2; //make sure we read only 2 bytes

		i2cTransaction.slaveAddress = i2c_addr;

		int ret = I2C_transfer(handle, &i2cTransaction);

		if (!ret) {
		    cli_printf("read16 error \n", 0);
		}else{
			returnValue = readBuffer[0];
			returnValue <<= 8;
			returnValue |= readBuffer[1];
		}

	}

	return returnValue;
}


void write8(unsigned char i2c_addr, I2C_Handle handle, uint8_t register_addr, uint8_t data) {

	I2C_Transaction i2cTransaction;
	uint8_t wb[2];
	wb[0] = register_addr;
	wb[1] = data;

	if(handle != NULL){

		i2cTransaction.readBuf = NULL;
		i2cTransaction.readCount = 0;
		i2cTransaction.writeBuf = wb;
		i2cTransaction.writeCount = 2;

		i2cTransaction.slaveAddress = i2c_addr;
		int ret = I2C_transfer(handle, &i2cTransaction);

		if (!ret) {
		    cli_printf("write8 error\n", 0);
		}
	}
}


