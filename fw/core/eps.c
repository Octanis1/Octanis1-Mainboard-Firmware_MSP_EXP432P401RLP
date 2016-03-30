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
#include "cli.h"

/* Commands */
// Switch modules on / off
#define M3V3_1_ON	0x31
#define M3V3_2_ON	0xD1
#define M3V3_1_OFF	0x30
#define M3V3_2_OFF	0xD0
#define M11V_ON		0xB1
#define M5V_ON		0x51
#define M11V_OFF    0xB0
#define M5V_OFF		0x50

#define SC_ON		0xC1
#define SC_OFF		0xC0

//switch heaters on / off
#define HEAT_1_ON	0x41
#define HEAT_1_OFF  0x40
#define HEAT_2_ON	0x43
#define HEAT_2_OFF  0x42
#define HEAT_3_ON	0x45
#define HEAT_3_OFF  0x44

// Confirmations (EPS responses)
#define COMM_OK				0x1A
#define LOW_VOLTAGE			0x99
#define UNKNOWN_COMMAND		0x77
#define COMM_ERROR			0xFA

// Ask for status data
#define V_BAT		0x0B
#define V_SC	    0x0C
#define I_IN		0x11
#define I_OUT		0x10
#define AEXT1		0xA1
#define AEXT2		0xA2
#define T_BAT		0xBB

// mainboard confirming "im still alive"
#define ALIVE 		0xA0

// define module states
#define OFF	0
#define ON	1


/* Struct definitions */
static struct _rover_status_eps {
	uint8_t	stat3V3_1;
	uint8_t	stat3V3_2;
	uint8_t	stat5V;
	uint8_t	stat11V;
	uint16_t v_bat;
	uint16_t v_solar;
	uint16_t i_in;
	uint16_t i_out;
} rover_status_eps;

static uint8_t give_life_sign;

void eps_init()
{
	rover_status_eps.stat3V3_1 = ON;  //GPS is active in the beginning
	rover_status_eps.stat3V3_2 = OFF;
	rover_status_eps.stat5V = OFF;
	rover_status_eps.stat11V = OFF;
	rover_status_eps.v_bat = 0;
	rover_status_eps.v_solar = 0;
	rover_status_eps.i_in = 0;
	rover_status_eps.i_out = 0;

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
			if(resp == COMM_OK) {return ON;}
			else if(resp == LOW_VOLTAGE) {
				cli_printf("Battery too low \n",0);
				return OFF;};
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

uint16_t eps_get_vbat()
{
	return rover_status_eps.v_bat;
}

uint16_t eps_get_vsolar()
{
	return rover_status_eps.v_solar;
}

void eps_task(){
	eps_init();
	give_life_sign = 0;

	while(1)
	{
		// check if we need to confirm that we are alive.
		if(give_life_sign)
		{
			sendEpsCommand(ALIVE);
			give_life_sign = 0;
		}

		// get status data (TODO: do correct conversion)
		rover_status_eps.v_bat = (uint16_t)sendEpsCommand(V_BAT);
		rover_status_eps.v_solar = (uint16_t)sendEpsCommand(V_SC);
		rover_status_eps.i_in = (uint16_t)sendEpsCommand(I_IN);
		rover_status_eps.i_out = (uint16_t)sendEpsCommand(I_OUT);


		Task_sleep(1000);

	}
}


void eps_ISR()
{
 //TODO
	give_life_sign = 1;

}
