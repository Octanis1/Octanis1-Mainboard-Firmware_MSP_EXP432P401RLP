/*
 * communication.c
 *
 *  Created on: 11 Mar 2016
 *      Author: raffael, beat
 */

#include <msp430.h>
#include "communication.h"
#include "commands.h"
#include "eps_hal.h"
#include "state_machine.h"

int mainboard_poke_iterate(int *mainboard_poke_counter)
{
	int i;
	if(++(*mainboard_poke_counter) == POKE_COUNTER_LIMIT)
	{
		//poke mainboard
		CLR_PIN(PORT_MB_POKE, PIN_MB_POKE);
		return 1;
	}
	if(++(*mainboard_poke_counter) == 2 * POKE_COUNTER_LIMIT)
	{
		//mainboard did not react to poke, cut power and delay for 5 timer ticks
		//cut mainboard power
		module_set_state(M_M, OFF);

		for(i = 0; i < 5; i++)
		{
			__bis_SR_register(CPUOFF + GIE);        // Enter LPM0 w/ interrupts
			__no_operation();                       // Set breakpoint >>here<< and read
		}
		//reset poke pin
		SET_PIN(PORT_MB_POKE, PIN_MB_POKE);
		//power up mainboard
		module_set_state(M_M, ON);
		//reset counter
		*mainboard_poke_counter = 0;
		return -1;
	}
	return 0;
}

void i2c_respond_command() // sets the response.
{
	if(i2c_available())
	{
		switch(i2c_read())
		{
			case ALIVE:
				i2c_send(COMM_OK);
				break;
			case M3V3_1_OFF:
				i2c_send(COMM_OK);
				module_status[M_331] = TURN_OFF;
				break;
			case M3V3_2_OFF:
				i2c_send(COMM_OK);
				module_status[M_332] = TURN_OFF;
				break;
			case M5V_OFF:
				i2c_send(COMM_OK);
				module_status[M_5] = TURN_OFF;
				break;
			case M11V_OFF:
				i2c_send(COMM_OK);
				module_status[M_11] = TURN_OFF;
				break;
			case HEAT_1_OFF:
				i2c_send(COMM_OK);
				module_status[H_T1] = TURN_OFF;
				break;
			case HEAT_2_OFF:
				i2c_send(COMM_OK);
				module_status[H_T2] = TURN_OFF;
				break;
			case HEAT_3_OFF:
				i2c_send(COMM_OK);
				module_status[H_T3] = TURN_OFF;
				break;
	//		case M3V3_1_ON: TXData = module_status[M331]; break;
	//		case M3V3_2_ON: TXData = module_status[M332]; break;
	//		case M5V_ON: TXData = module_status[M5]; break;
	//		case M11V_ON: TXData = module_status[M11]; break;
	//		case HEAT_1_ON: TXData=module_status[HT1]; break;
	//		case HEAT_2_ON: TXData=module_status[HT2]; break;
	//		case HEAT_3_ON: TXData=module_status[HT3]; break;

	//		case V_BAT:TXData = eps_status.v_bat_8; break;
	//		case V_SC: TXData = eps_status.v_solar_8; break;
	//		case I_IN: TXData = eps_status.current_in_8; break;
	//		case I_OUT: TXData = eps_status.current_out_8; break;
	//		case AEXT1: TXData = eps_status.analog_ext1_8; break;
	//		case AEXT2: TXData = eps_status.analog_ext2_8; break;
	//		case T_BAT:TXData = eps_status.t_bat_8;break;

	//		#ifndef ANALOG_6
	//		case AEXT3: TXData = eps_status.analog_ext3; break;
	//		case AEXT4: TXData = eps_status.analog_ext4; break;
	//		#endif

			default: i2c_send(UNKNOWN_COMMAND);break;
		}
	}
}

void execute_i2c_command(unsigned char command)
{
		switch(command)
		{
			default: break;
		}

}
