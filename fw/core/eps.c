/*
 * eps.c
 *
 * Interface to the electrical power subsystem (EPS)
 *
 *  Created on: 08 Mar 2016
 *      Author: raffael
 */


#include "../../Board.h"
#include "../peripherals/hal/i2c_helper.h"
#include "eps.h"

/* Commands */

// Confirmations (EPS responses)
#define EPS_OK				0x1A
#define LOW_VOLTAGE			0x99
#define UNKNOWN_COMMAND		0x77
#define EPS_ERROR			0xFA

// Ask for status data
#define V_BAT		0x0B
#define V_SC			0x0C
#define I_IN			0x11
#define I_OUT		0x10

/* Struct definitions */
static struct _rover_status_eps {
	uint8_t	stat3V3_1;
	uint8_t	stat3V3_2;
	uint8_t	stat5V;
	uint8_t	stat11V;
	uint16_t v_bat;
	uint16_t v_solar;
} rover_status_eps;

void eps_init()
{
	rover_status_eps.stat3V3_1 = OFF;
	rover_status_eps.stat3V3_2 = OFF;
	rover_status_eps.stat5V = OFF;
	rover_status_eps.stat11V = OFF;
	rover_status_eps.v_bat = 0;
	rover_status_eps.v_solar = 0;

	i2c_helper_init_handle();
	//GPIO_enableInt(Board_LIGHTNING_INT)
}

uint8_t sendEpsCommand(uint8_t command)
{
	I2C_Transaction i2cTransaction;

	int8_t iError = 0;
	uint8_t readBuffer;

	i2cTransaction.writeBuf = &command;
	i2cTransaction.writeCount = sizeof(command);

	i2cTransaction.readBuf = &readBuffer;
	i2cTransaction.readCount = sizeof(readBuffer);

	i2cTransaction.slaveAddress = Board_EPS_I2CADDR;

	int ret = I2C_transfer(i2c_helper_handle, &i2cTransaction);

	if (!ret) {
		iError = -1;
	}else{
		iError = 0;
	}

	return readBuffer;
}

uint8_t eps_switch_module(uint8_t command) //use commands defined in eps.h
{
	static uint8_t resp = 0;
	static uint8_t i=0;

	if(command & 0x01) //turn on a module
	{
		for(i=0; i<3; i++)
		{
			resp = sendEpsCommand(command);
			if(resp == EPS_OK) {return ON;}
			else if(resp == LOW_VOLTAGE) {return OFF};
			//else try again or abandon after 3 times
		}
		return OFF; // = error = 0
	}
	else //turn off a module
	{
		resp = sendEpsCommand(command);
		return OFF; // = error = 0
	}

}


void eps_ISR()
{
 //TODO

}
