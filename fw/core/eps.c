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
#include "../peripherals/comm.h"

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
	GPIO_enableInt(Board_EPS_ALIVE_REQ);
}

COMM_FRAME* eps_pack_mavlink_battery_status()
{
	// Initialize the message buffer
	static COMM_FRAME frame;

	uint16_t vbat[10] = {UINT16_MAX,};
	vbat[0] = rover_status_eps.v_bat;
	int32_t current_consumed = -1; //Consumed charge, in milliampere hours (1 = 1 mAh), -1: autopilot does not provide mAh consumption estimate
	int16_t current_battery = rover_status_eps.i_out; //Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
	int32_t energy_consumed = -1;  //Consumed energy, in 100*Joules (intergrated U*I*dt) (1 = 100 Joule), -1: autopilot does not provide energy consumption estimate
	int8_t battery_remaining = -1;	// Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot does not estimate the remaining battery

	mavlink_msg_battery_status_pack(mavlink_system.sysid, MAV_COMP_ID_SYSTEM_CONTROL, &(frame.mavlink_message),
		0, MAV_BATTERY_FUNCTION_ALL, MAV_BATTERY_TYPE_LION, UINT16_MAX, vbat, current_battery, current_consumed, energy_consumed, battery_remaining);

	//todo: correct battery temperature!

	return &frame;
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

uint16_t readEpsReg(uint8_t reg)
{
	I2C_Transaction i2cTransaction;

	int8_t iError = 0;
	uint16_t readBuffer;

	i2cTransaction.writeBuf = &reg;
	i2cTransaction.writeCount = sizeof(reg);

	i2cTransaction.readBuf = &readBuffer;
	i2cTransaction.readCount = sizeof(readBuffer);

	i2cTransaction.slaveAddress = Board_EPS_I2CADDR;

	int ret = I2C_transfer(i2c_helper_handle, &i2cTransaction);

	if (!ret) {
		iError = -1;
		serial_printf(cli_stdout, "transaction failed %d \r\n",ret);

	}else{
		iError = 0;

		serial_printf(cli_stdout, "read: %u \r\n",readBuffer);

	}

	return readBuffer;
}

uint8_t eps_switch_module(uint8_t command) //use commands defined in eps.h
{
#ifdef EPS_ENABLED
	static uint8_t resp = 0;
	static uint8_t i=0;

	if(command & 0x01) //turn on a module
	{
		for(i=0; i<3; i++)
		{
			resp = sendEpsCommand(command);
			if(resp == COMM_OK) {return ON;}
			else if(resp == LOW_VOLTAGE) {
				serial_printf(cli_stdout, "Battery too low \r\n",0);
				return OFF;};
			//else try again or abandon after 3 times
		}
		serial_printf(cli_stdout, "EPS comm err \r\n",0);
		return OFF; // = error = 0
	}
	else //turn off a module
	{
		resp = sendEpsCommand(command);
		return OFF; // = error = 0
	}
#else
	return ON;
#endif
}

uint16_t eps_get_vbat()
{
	return rover_status_eps.v_bat;
}

uint16_t eps_get_vsolar()
{
	return rover_status_eps.v_solar;
}

uint16_t eps_get_iin()
{
	return rover_status_eps.i_in;
}

uint16_t eps_get_iout()
{
	return rover_status_eps.i_out;
}

void eps_task(){

#ifndef MAVLINK_ON_UART0_ENABLED
	cli_init();
#endif

#ifdef EPS_ENABLED
	eps_init();
	give_life_sign = 1;
#endif
	while(1)
	{
#ifdef EPS_ENABLED

		// check if we need to confirm that we are alive.
		if(give_life_sign)
		{
			sendEpsCommand(ALIVE);
			give_life_sign = 0;
		}

		// get status data (TODO: do correct conversion)
		rover_status_eps.v_bat = (uint16_t)((float)readEpsReg(V_BAT)*7.026+2582); //<-- flight version Payload1 calibrated value;  was set to: (float)sendEpsCommand(V_BAT)*6.67+2500)
		rover_status_eps.v_solar = (uint16_t)((float)readEpsReg(V_SC)*29.23); //flight version was wrongly *27.5, should have been 29.23 for 1.2M
		rover_status_eps.i_in = (uint16_t)((float)readEpsReg(I_IN)*1.19); //
		rover_status_eps.i_out = (uint16_t)((float)readEpsReg(I_OUT)*4.76);// depends on current sense resistor on eps!

		eps_pack_mavlink_battery_status();


		Task_sleep(500);

		// check if we need to confirm that we are alive. (do it twice per second)
		if(give_life_sign)
		{
			sendEpsCommand(ALIVE);
			give_life_sign = 0;
		}
#endif
		Task_sleep(500);


	}
}


void eps_ISR()
{
 //TODO
	give_life_sign = 1;
}
