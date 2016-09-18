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
#include <math.h>

#define HEARTBEAT_TO_MEASUREMENT_RATIO	20 // Heartbeats to EPS are sent every 500ms. Battery status is asked every *ratio* times

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
	int16_t t_bat; // in 0.01 degrees celsius
} rover_status_eps;

static struct _rover_status_eps_extrema{
	uint16_t v_bat_min;
	uint16_t v_bat_max;
	uint16_t v_solar_min;
	uint16_t v_solar_max;
	uint16_t i_in_max;
	uint16_t i_out_max;
} rover_status_eps_extrema;

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

	rover_status_eps_extrema.v_bat_min = UINT16_MAX;
	rover_status_eps_extrema.v_bat_max = 0;
	rover_status_eps_extrema.v_solar_min = UINT16_MAX;
	rover_status_eps_extrema.v_solar_max = 0;
	rover_status_eps_extrema.i_in_max = 0;
	rover_status_eps_extrema.i_out_max = 0;

	i2c_helper_init_handle();
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
	uint16_t drop_rate_comm = cli_mavlink_dropcount(); //Communication drops in percent, (0%: 0, 100%: 10'000), (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
	uint16_t errors_comm=0; //Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)

	mavlink_msg_sys_status_pack(mavlink_system.sysid, MAV_COMP_ID_SYSTEM_CONTROL, &(frame.mavlink_message),
		   onboard_control_sensors_present, onboard_control_sensors_enabled, onboard_control_sensors_health, load,
		   rover_status_eps.v_bat, (rover_status_eps.i_out/10), battery_remaining, drop_rate_comm, errors_comm, 0,0,0,0); //WARNING: iout in miliamps is not according to standard

	return &frame;
}

COMM_FRAME* eps_pack_mavlink_battery_status()
{
	// Initialize the message buffer
	static COMM_FRAME frame;

	static uint16_t vbat[10] = {UINT16_MAX, };
	vbat[0] = rover_status_eps.v_bat;
	vbat[1] = rover_status_eps.v_solar;
	vbat[2] = rover_status_eps.i_in;
	vbat[3] = rover_status_eps_extrema.v_bat_min;
	vbat[4] = rover_status_eps_extrema.v_bat_max;
	vbat[5] = rover_status_eps_extrema.v_solar_min;
	vbat[6] = rover_status_eps_extrema.v_solar_max;
	vbat[7] = rover_status_eps_extrema.i_out_max;
	vbat[8] = rover_status_eps_extrema.i_in_max;

	int32_t current_consumed = -1; //Consumed charge, in milliampere hours (1 = 1 mAh), -1: autopilot does not provide mAh consumption estimate
	int32_t current_battery = rover_status_eps.i_out/10; //Battery current, in milliamperes (1 = 1 milliampere), -1: autopilot does not measure the current !!!WARNING: this is not according to mavlink standard
	int32_t energy_consumed = -1;  //Consumed energy, in 100*Joules (intergrated U*I*dt) (1 = 100 Joule), -1: autopilot does not provide energy consumption estimate
	int8_t battery_remaining = (rover_status_eps.v_bat - 3000)/11;	// Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot does not estimate the remaining battery

	mavlink_msg_battery_status_pack(mavlink_system.sysid, MAV_COMP_ID_SYSTEM_CONTROL, &(frame.mavlink_message),
		0, MAV_BATTERY_FUNCTION_ALL, MAV_BATTERY_TYPE_LION, rover_status_eps.t_bat, vbat, (int16_t)current_battery, current_consumed, energy_consumed, battery_remaining);

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
		return UINT16_MAX;
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
	sendEpsCommand(ALIVE);


#endif
	while(1)
	{
#ifdef EPS_ENABLED

		// get status data (TODO: do correct conversion)
		rover_status_eps.v_bat = readEpsReg(V_BAT);
		rover_status_eps.v_solar = readEpsReg(V_SC);
		rover_status_eps.i_in = readEpsReg(I_IN);
		rover_status_eps.i_out = readEpsReg(I_OUT);

		uint16_t R_th = readEpsReg(T_BAT);

		if(R_th != UINT16_MAX)
		{
			const static double A = 1.129148e-3;
			const static double B = 2.34125e-4;
			const static double C = 8.76741e-8;

			double logR  = log((double)R_th);
			double logR3 = logR * logR * logR;

			rover_status_eps.t_bat = (int16_t)(100/(A + B * logR + C * logR3 ) - 273.15);
		}

		rover_status_eps_extrema.v_bat_min = (rover_status_eps_extrema.v_bat_min > rover_status_eps.v_bat) ? rover_status_eps.v_bat : rover_status_eps_extrema.v_bat_min;
		rover_status_eps_extrema.v_bat_max = (rover_status_eps_extrema.v_bat_max < rover_status_eps.v_bat) ? rover_status_eps.v_bat : rover_status_eps_extrema.v_bat_max;
		rover_status_eps_extrema.v_solar_min = (rover_status_eps_extrema.v_solar_min > rover_status_eps.v_solar) ? rover_status_eps.v_solar : rover_status_eps_extrema.v_solar_min;
		rover_status_eps_extrema.v_solar_max = (rover_status_eps_extrema.v_solar_max < rover_status_eps.v_solar) ? rover_status_eps.v_solar : rover_status_eps_extrema.v_solar_max;
		rover_status_eps_extrema.i_in_max =  (rover_status_eps_extrema.i_in_max < rover_status_eps.i_in) ? rover_status_eps.i_in : rover_status_eps_extrema.i_in_max;
		rover_status_eps_extrema.i_out_max =  (rover_status_eps_extrema.i_out_max < rover_status_eps.i_out) ? rover_status_eps.i_out : rover_status_eps_extrema.i_out_max;

		comm_mavlink_broadcast(eps_pack_mavlink_battery_status());

		comm_mavlink_broadcast(eps_pack_mavlink_sys_status());

		int i;
		for(i= 0; i< HEARTBEAT_TO_MEASUREMENT_RATIO; i++)
		{
			Task_sleep(500);
			// heartbeat to EPS
			sendEpsCommand(ALIVE);
		}
#endif
		Task_sleep(200);
	}
}
