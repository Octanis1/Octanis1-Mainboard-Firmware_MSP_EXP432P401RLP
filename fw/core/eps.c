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
#include <ti/sysbios/utils/Load.h>

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

const static uint32_t onboard_control_sensors_present = 	MAV_SYS_STATUS_SENSOR_3D_GYRO +
														MAV_SYS_STATUS_SENSOR_3D_ACCEL+
														MAV_SYS_STATUS_SENSOR_3D_MAG +
														MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE +
														MAV_SYS_STATUS_SENSOR_GPS +
														MAV_SYS_STATUS_SENSOR_YAW_POSITION +
														MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;

COMM_FRAME* eps_pack_mavlink_sys_status()
{
	// Initialize the message buffer
	static COMM_FRAME frame;

	uint32_t onboard_control_sensors_enabled = onboard_control_sensors_present; //TODO
	uint32_t onboard_control_sensors_health = onboard_control_sensors_present; //TODO
	uint16_t load = (uint16_t)10*Load_getCPULoad(); //Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
	int8_t battery_remaining = (rover_status_eps.v_bat - 3000)/11;	// Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot does not estimate the remaining battery
	uint16_t drop_rate_comm=0; //Communication drops in percent, (0%: 0, 100%: 10'000), (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
	uint16_t errors_comm=0; //Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)

	mavlink_msg_sys_status_pack(mavlink_system.sysid, MAV_COMP_ID_SYSTEM_CONTROL, &(frame.mavlink_message),
		   onboard_control_sensors_present, onboard_control_sensors_enabled, onboard_control_sensors_health, load,
		   rover_status_eps.v_bat, rover_status_eps.i_out, battery_remaining, drop_rate_comm, errors_comm, 0,0,0,0);

	return &frame;
}

COMM_FRAME* eps_pack_mavlink_battery_status()
{
	// Initialize the message buffer
	static COMM_FRAME frame;

	static uint16_t vbat[10] = {UINT16_MAX,UINT16_MAX,UINT16_MAX,UINT16_MAX,UINT16_MAX,UINT16_MAX,UINT16_MAX,UINT16_MAX,UINT16_MAX,UINT16_MAX};
	vbat[0] = rover_status_eps.v_bat;
	int32_t current_consumed = -1; //Consumed charge, in milliampere hours (1 = 1 mAh), -1: autopilot does not provide mAh consumption estimate
	int16_t current_battery = rover_status_eps.i_out; //Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
	int32_t energy_consumed = -1;  //Consumed energy, in 100*Joules (intergrated U*I*dt) (1 = 100 Joule), -1: autopilot does not provide energy consumption estimate
	int8_t battery_remaining = (rover_status_eps.v_bat - 3000)/11;	// Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot does not estimate the remaining battery

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
	int16_t readBuffer;

	i2cTransaction.writeBuf = &reg;
	i2cTransaction.writeCount = sizeof(reg);

	i2cTransaction.readBuf = &readBuffer;
	i2cTransaction.readCount = sizeof(readBuffer);

	i2cTransaction.slaveAddress = Board_EPS_I2CADDR;

	int ret = I2C_transfer(i2c_helper_handle, &i2cTransaction);

	if (!ret) {
		iError = -1;
		return -1;
//		serial_printf(cli_stdout, "EPS transaction failed %d \r\n",ret);

	}else{
		iError = 0;

//		serial_printf(cli_stdout, "read: %u \r\n",readBuffer);

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
		rover_status_eps.v_bat = readEpsReg(V_BAT);
		rover_status_eps.v_solar = readEpsReg(V_SC);
		rover_status_eps.i_in = readEpsReg(I_IN);
		rover_status_eps.i_out = readEpsReg(I_OUT);

#ifdef MAVLINK_ON_UART0_ENABLED
		comm_set_tx_flag(CHANNEL_APP_UART, MAV_COMP_ID_SYSTEM_CONTROL);
#endif
		comm_mavlink_broadcast(eps_pack_mavlink_battery_status());

#ifdef MAVLINK_ON_UART0_ENABLED
		comm_set_tx_flag(CHANNEL_APP_UART, MAV_COMP_ID_SYSTEM_CONTROL);
#endif
		comm_mavlink_broadcast(eps_pack_mavlink_sys_status());



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
