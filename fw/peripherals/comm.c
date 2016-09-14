/*
 * comm.c
 *
 *  Created on: 10.08.2015
 *      Author: Sam
 */

#include "../../Board.h"
#include "comm.h"
#include <string.h>
#include "hal/time_since_boot.h"

#include "navigation.h"
#include "imu.h"
#include "rockblock.h"

#define MSG_ID_FIELD				N_COMM_CHANNELS

#define N_PERIODIC_MAVLINK_MSG	13
#define P_INF					UINT32_MAX
static const uint32_t mavlink_message_periods[N_PERIODIC_MAVLINK_MSG][N_COMM_CHANNELS+1] = {
/* Definitions of the minimal time in ms between two consecutive message transmissions.
 * --> only applies to periodically sent messages (i.e. measurements, status updates, etc.)
 *
 *	UART			LORA			ROCKBLOCK	GSM		| MESSAGE_ID */
	{5000,		10000,		600000,		P_INF,	MAVLINK_MSG_ID_HEARTBEAT},
	{10000,		30000,		600000,		P_INF,	MAVLINK_MSG_ID_SYS_STATUS},
	{0,			30000,		600000,		P_INF,	MAVLINK_MSG_ID_GPS_RAW_INT},
	{200,		30000,		600000,		P_INF,	MAVLINK_MSG_ID_GLOBAL_POSITION_INT},
	{1000,		30000,		P_INF,		P_INF,	MAVLINK_MSG_ID_SCALED_IMU},
	{500,		30000,		600000,		P_INF,	MAVLINK_MSG_ID_SCALED_PRESSURE},
	{500,		30000,		600000,		P_INF,	MAVLINK_MSG_ID_ATTITUDE},
	{1000,		30000,		P_INF,		P_INF,	MAVLINK_MSG_ID_RC_CHANNELS_SCALED},
	{500,		30000,		600000,		P_INF,	MAVLINK_MSG_ID_VFR_HUD},
	{30000,		P_INF,		P_INF,		P_INF,	MAVLINK_MSG_ID_RADIO_STATUS},
	{30000,		30000,		600000,		P_INF,	MAVLINK_MSG_ID_BATTERY_STATUS},
	{P_INF,		P_INF,		P_INF,		P_INF,	MAVLINK_MSG_ID_WIND_COV},
	{10000,		30000,		600000,		P_INF,	MAVLINK_MSG_ID_RC_CHANNELS},
};

static uint32_t mavlink_periodic_msg_tx_times[N_PERIODIC_MAVLINK_MSG][N_COMM_CHANNELS]= {{0,},};

/* this function pre-fills the time stamp array of the mavlink broadcasts.
 * Goal:
 * distribute the TX events more evenly in order to prevent mailbox overflow.
 */
void comm_add_offset_to_mavlink_tx_times()
{
	/*LoRa*/
	mavlink_periodic_msg_tx_times[MAVLINK_MSG_ID_BATTERY_STATUS][CHANNEL_LORA] = 5000;
	mavlink_periodic_msg_tx_times[MAVLINK_MSG_ID_SYS_STATUS][CHANNEL_LORA] = 2000;
	mavlink_periodic_msg_tx_times[MAVLINK_MSG_ID_GPS_RAW_INT][CHANNEL_LORA] = 10000;
	mavlink_periodic_msg_tx_times[MAVLINK_MSG_ID_GLOBAL_POSITION_INT][CHANNEL_LORA] = 12000;
	mavlink_periodic_msg_tx_times[MAVLINK_MSG_ID_SCALED_IMU][CHANNEL_LORA] = 15000;
	mavlink_periodic_msg_tx_times[MAVLINK_MSG_ID_SCALED_PRESSURE][CHANNEL_LORA] = 17000;
	mavlink_periodic_msg_tx_times[MAVLINK_MSG_ID_ATTITUDE][CHANNEL_LORA] = 19000;
	mavlink_periodic_msg_tx_times[MAVLINK_MSG_ID_SCALED_PRESSURE][MAVLINK_MSG_ID_VFR_HUD] = 21000;
	mavlink_periodic_msg_tx_times[MAVLINK_MSG_ID_RC_CHANNELS][CHANNEL_LORA] = 22000;
	mavlink_periodic_msg_tx_times[MAVLINK_MSG_ID_RC_CHANNELS_SCALED][CHANNEL_LORA] = 24000;

	/*Rockblock*/
	// SMALLEST PRIORITY: sensor data
	mavlink_periodic_msg_tx_times[MAVLINK_MSG_ID_SCALED_PRESSURE][CHANNEL_ROCKBLOCK] = 400000;
	mavlink_periodic_msg_tx_times[MAVLINK_MSG_ID_ATTITUDE][CHANNEL_ROCKBLOCK] = 400000;
	mavlink_periodic_msg_tx_times[MAVLINK_MSG_ID_RC_CHANNELS][CHANNEL_ROCKBLOCK] = 400000;
	// second smallest prio: GPS data (may have to wait for survey-in
	mavlink_periodic_msg_tx_times[MAVLINK_MSG_ID_GPS_RAW_INT][CHANNEL_ROCKBLOCK] = 200000;
	mavlink_periodic_msg_tx_times[MAVLINK_MSG_ID_GLOBAL_POSITION_INT][CHANNEL_ROCKBLOCK] = 200000;


}

#define N_IRREGULAR_MAVLINK_MSG	14
static const uint8_t mavlink_message_allow_tx[N_IRREGULAR_MAVLINK_MSG][N_COMM_CHANNELS+1] = {
/* Definitions of the minimal time in ms between two consecutive message transmissions.
 * --> only applies to periodically sent messages (i.e. measurements, status updates, etc.)
 *
 *	UART		LORA		RCKBLK	GSM		| MESSAGE_ID */
	{0,		1,		0,		0,		MAVLINK_MSG_ID_SET_MODE},
	{1,		0,		0,		0,		MAVLINK_MSG_ID_MISSION_ITEM},
	{0,		0,		1,		0,		MAVLINK_MSG_ID_MISSION_ITEM},
	{1,		0,		0,		0,		MAVLINK_MSG_ID_MISSION_REQUEST},
	{0,		1,		0,		0,		MAVLINK_MSG_ID_MISSION_SET_CURRENT},
	{1,		1,		1,		0,		MAVLINK_MSG_ID_MISSION_CURRENT},
	{1,		0,		0,		0,		MAVLINK_MSG_ID_MISSION_REQUEST_LIST},
	{1,		0,		0,		0,		MAVLINK_MSG_ID_MISSION_COUNT},
	{1,		1,		1,		0,		MAVLINK_MSG_ID_MISSION_ITEM_REACHED},
	{1,		0,		0,		0,		MAVLINK_MSG_ID_MISSION_ACK},
	{1,		1,		0,		0,		MAVLINK_MSG_ID_COMMAND_LONG},
	{1,		1,		0,		0,		MAVLINK_MSG_ID_COMMAND_ACK},
	{1,		0,		0,		0,		MAVLINK_MSG_ID_ENCAPSULATED_DATA},
	{1,		0,		0,		0,		MAVLINK_MSG_ID_STATUSTEXT},
};


mavlink_system_t mavlink_system;
static mavlink_heartbeat_t mavlink_heartbeat;
mavlink_heartbeat_t comm_get_mavlink_heartbeat(){return mavlink_heartbeat;}

char comm_mainboard_armed()
{return mavlink_heartbeat.system_status == MAV_STATE_ACTIVE;}

void comm_arm_mainboard() {mavlink_heartbeat.system_status = MAV_STATE_ACTIVE; mavlink_heartbeat.base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;}
void comm_disarm_mainboard() {mavlink_heartbeat.system_status = MAV_STATE_STANDBY; mavlink_heartbeat.base_mode &= ~MAV_MODE_FLAG_SAFETY_ARMED;}

// We periodically check if we still get heartbeat messages from the SBC
static int32_t t_last_sbc_heartbeat;
static mavlink_heartbeat_t sbc_heartbeat;
static float sbc_last_command_arm; //temporarily stores if the last command was arm or disarm. -1 if no command was sent
#define SBC_HEARTBEAT_TIMEOUT 10

char comm_sbc_running(){
//	return 1; //TODO: remove this line!!
	return t_last_sbc_heartbeat > ((int32_t)Seconds_get() - (int32_t)SBC_HEARTBEAT_TIMEOUT);}
char comm_sbc_armed(){
	return (comm_sbc_running()==1 && sbc_heartbeat.system_status == MAV_STATE_ACTIVE);}

void comm_mavlink_post_outbox(COMM_CHANNEL channel, COMM_FRAME* frame); //post to mailbox for outgoing messages
void comm_clear_tx_flag(COMM_CHANNEL channel, int component_id);

/* PUBLIC */

uint8_t comm_mavlink_broadcast(COMM_FRAME* frame) //posts to mailbox for all available channel slots for a given component
{
	int per_id, irr_id = 0;
	uint8_t retval = BROADCAST_SUCCESS;

	//search for the index of the message id in periodic messages set:
	for(per_id = 0; per_id<N_PERIODIC_MAVLINK_MSG; per_id++)
	{
		if((frame->mavlink_message.msgid) == mavlink_message_periods[per_id][MSG_ID_FIELD])
		{//found the index
			int ch_id;
			for(ch_id=0;ch_id<N_COMM_CHANNELS;ch_id++)
			{
				if(ms_since_boot() - mavlink_periodic_msg_tx_times[per_id][ch_id] > mavlink_message_periods[per_id][ch_id])
				{
					comm_mavlink_post_outbox(ch_id, frame);
					mavlink_periodic_msg_tx_times[per_id][ch_id] += mavlink_message_periods[per_id][ch_id];
					retval|=1<<ch_id;
				}
			}
			if(retval == 0)
				retval = BROADCAST_ALL_BUSY;
			return retval;
		}
	}

	//search for the index of the message id in irregular messages set:
	for(irr_id = 0; irr_id<N_IRREGULAR_MAVLINK_MSG; irr_id++)
	{
		if(frame->mavlink_message.msgid == mavlink_message_allow_tx[irr_id][MSG_ID_FIELD])
		{//found the index
			int ch_id;
			for(ch_id=0;ch_id<N_COMM_CHANNELS;ch_id++)
			{
				if(mavlink_message_allow_tx[irr_id][ch_id])
				{
					comm_mavlink_post_outbox(ch_id, frame);
					retval|=1<<ch_id;
				}
			}
			if(retval == 0)
				retval = BROADCAST_IRR_MSG_NOT_IN_SET;
			return retval;
		}
	}

	if(irr_id == N_IRREGULAR_MAVLINK_MSG && per_id == N_PERIODIC_MAVLINK_MSG)
	{
		serial_printf(cli_stdout,"Failed attempt to send mavlink message. MSG ID unknown \n");
		return BROADCAST_MSG_UNKNOWN;
	}
	else
	{
		serial_printf(cli_stdout,"ERROR in broadcasting fx \n");
		return BROADCAST_ERROR;
	}
}

void comm_mavlink_post_inbox(COMM_CHANNEL channel, mavlink_message_t *message); //post to mailbox for incoming messages

/* PRIVATE */

void comm_mavlink_post_outbox(COMM_CHANNEL channel, COMM_FRAME* frame) //post to mailbox for outgoing messages
{
	frame->channel = channel;
	frame->direction = CHANNEL_OUT;

	switch(channel)
	{
		case CHANNEL_APP_UART:
	#ifdef MAVLINK_ON_UART0_ENABLED
	#ifdef SBC_ENABLED
			if(sbc_heartbeat.system_status == MAV_STATE_STANDBY || sbc_heartbeat.system_status == MAV_STATE_ACTIVE)
	#endif
			{
				Mailbox_post(comm_mailbox, frame, BIOS_NO_WAIT);
			}
	#endif
			break;
		case CHANNEL_LORA:
	#ifdef LORA_ENABLED
			Mailbox_post(lora_mailbox, frame, BIOS_NO_WAIT);
	#endif
			break;
		case CHANNEL_ROCKBLOCK:
	#ifdef ROCKBLOCK_ENABLED
			Mailbox_post(rockblock_mailbox, frame, BIOS_NO_WAIT);
	#endif
			break;
		case CHANNEL_GSM:
			   //sim800_send_http(msg)
		default:
			break;
	}
}

void comm_uart_send(COMM_CHANNEL channel, mavlink_message_t *msg){

	static uint8_t buf[MAVLINK_MAX_PACKET_LEN + 5]; //todo: +5 is to test if we need some more space. remove if not helpful.
	static uint16_t mavlink_msg_len;

	// Copy the message to the send buffer and send
	mavlink_msg_len = mavlink_msg_to_send_buffer(buf, msg);

	if(channel==CHANNEL_APP_UART)
	{
#ifdef MAVLINK_ON_UART0_ENABLED
#ifdef SBC_ENABLED
			if(sbc_heartbeat.system_status == MAV_STATE_STANDBY || sbc_heartbeat.system_status == MAV_STATE_ACTIVE)
#endif
			{
				serial_write(cli_stdout, buf, mavlink_msg_len);
			}
#endif

	}
	else
	{
		serial_printf(cli_stdout,"ERROR: outgoing msg for channel %d in uart_mailbox.id=%d \n", channel, msg->msgid);
	}
}

/**********************************************************************************
 **********************************************************************************
 *********** Functions relative to MAVlink message processing *********************
 **********************************************************************************
 **********************************************************************************
 */

// currently only sends arm command to SBC:
void comm_arm_disarm_subsystems(float arm_disarm)
{
	COMM_FRAME frame;

	// this function prepares an ARM or DISARM command to be sent to the SBC.
	mavlink_msg_command_long_pack(mavlink_system.sysid, MAV_COMP_ID_ALL, &(frame.mavlink_message),
	SBC_SYSTEM_ID, MAV_COMP_ID_ALL, MAV_CMD_COMPONENT_ARM_DISARM, 0, arm_disarm, 0, 0, 0, 0, 0, 0);

	sbc_last_command_arm = arm_disarm;
	comm_mavlink_post_outbox(CHANNEL_APP_UART, &frame);
}

COMM_MAV_RESULT comm_process_command(COMM_MAV_MSG_TARGET*  msg_target, mavlink_message_t *msg, mavlink_message_t *answer_msg)
{	//TODO
	uint16_t command = mavlink_msg_command_long_get_command(msg);
	uint8_t confirmation = mavlink_msg_command_long_get_confirmation(msg);
	MAV_RESULT result = MAV_RESULT_DENIED;
	switch (command)
	{
		case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN: //#246 --> shut down SBC. forward command to olimex.
			// TODO: shut down SBC
			{
				mavlink_msg_command_long_pack(mavlink_system.sysid, msg_target->component, answer_msg,
				SBC_SYSTEM_ID, msg_target->component, MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, confirmation, 0, 0, 0, 0, 0, 0, 0);

				msg_target->system = SBC_SYSTEM_ID; //target system of the to be forwarded message.

				return FORWARD_MESSAGE;}
			break;
		case MAV_CMD_NAV_LAND: //#21 [Land]
		{
			if(navigation_bypass('b',0))
				result = MAV_RESULT_ACCEPTED;
			break;
		}
		case MAV_CMD_NAV_TAKEOFF: //#22 [Start]
		{
			if(navigation_bypass('f',0))
				result = MAV_RESULT_ACCEPTED;
			break;
		}
		case MAV_CMD_OVERRIDE_GOTO: //#252 Hold / continue the current action [halt] --> used to pause/start driving
		{
			result = navigation_halt_resume(msg_target, msg);
			break;
		}
		case MAV_CMD_COMPONENT_ARM_DISARM: //#400 Arms / Disarms a component --- param1: 1 to arm, 0 to disarm
			//forward to SBC board, if needed
			{
				navigation_rxcmd_arm_disarm(mavlink_msg_command_long_get_param1(msg));
				result = MAV_RESULT_ACCEPTED;

				break;
			}
		case MAV_CMD_SET_MESSAGE_INTERVAL:
		case MAV_CMD_GET_MESSAGE_INTERVAL:
		default:
			result = MAV_RESULT_UNSUPPORTED;
			break;
	}

	mavlink_msg_command_ack_pack(mavlink_system.sysid, msg_target->component, answer_msg,
			command, result);

	return REPLY_TO_SENDER;
}

//returns true if message target is this system --> you can then continue decoding the message.
//otherwise it should forward the message to the corresponding target (i.e. EPS, SBC, etc...)
int comm_mavlink_check_target(COMM_MAV_MSG_TARGET* target, mavlink_message_t *msg)
{
	if(target->system == mavlink_system.sysid)
	{
		return 1;
	}
	else if(target->system == SBC_SYSTEM_ID)
	{
		COMM_FRAME forward_frame;
		(forward_frame.mavlink_message) = (*msg);

		// forward to SBC connected to UART0
		comm_mavlink_post_outbox(CHANNEL_APP_UART, &forward_frame);
		//TODO: forward the message to corresponding system

	}

	return 0;
}



void comm_mavlink_handler(COMM_CHANNEL src_channel, mavlink_message_t *msg){
	COMM_MAV_MSG_TARGET msg_target;
	COMM_FRAME answer_frame; // This variable must be filled by any external function that handles
							// 	a mavlink message, if it wants to send back an answer to the sender.
	COMM_MAV_RESULT mav_result = NO_ANSWER; // should be the return value of every external function that handles
							// 	a mavlink message, and set to 'REPLY_TO_SENDER' if it wants to send an answer.

	if(src_channel == CHANNEL_LORA)
		serial_printf(cli_stdout, "LoRa msg arrived in mailbox: ID=%d\r\n", msg->msgid);


	switch(msg->msgid){
		case MAVLINK_MSG_ID_HEARTBEAT:
		{
			if(msg->sysid == SBC_SYSTEM_ID)
			{
				// TODO: check if heartbeat really changes its mode when armed.
				// 			otherwise just change manually upon reception of COMMAND_ACK message
//				mavlink_msg_heartbeat_decode(msg, &sbc_heartbeat);
				if(sbc_heartbeat.system_status < MAV_STATE_STANDBY)
					sbc_heartbeat.system_status = MAV_STATE_STANDBY; //just to be sure to change it from booting state.
				t_last_sbc_heartbeat = (int32_t)Seconds_get();
			}
			break;
		}
		case MAVLINK_MSG_ID_COMMAND_LONG:
		{
			msg_target.system = mavlink_msg_command_long_get_target_system(msg);
			msg_target.component = mavlink_msg_command_long_get_target_component(msg);

			if(comm_mavlink_check_target(&msg_target,msg))
			{
				mav_result = comm_process_command(&msg_target, msg, &(answer_frame.mavlink_message));
			}
			break;
		}
		case MAVLINK_MSG_ID_COMMAND_ACK:
		{
			// If acknowledge for shutdown of sbc, forward it to APM planner.
			uint16_t command = mavlink_msg_command_ack_get_command(msg);
			uint8_t result = mavlink_msg_command_ack_get_result(msg);
			if(msg->sysid == SBC_SYSTEM_ID && command == MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN)
			{
				mavlink_msg_command_ack_pack(mavlink_system.sysid, 1, &(answer_frame.mavlink_message), //cmp id is usually 1 when receiving the command.
						command, result);
			}
			else if(msg->sysid == SBC_SYSTEM_ID && command == MAV_CMD_COMPONENT_ARM_DISARM)
			{
				if(sbc_last_command_arm == 1)
					sbc_heartbeat.system_status = MAV_STATE_ACTIVE;
				else if(sbc_last_command_arm == 0)
					sbc_heartbeat.system_status = MAV_STATE_STANDBY;
			}
			else if(msg->sysid == SBC_SYSTEM_ID && command == MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN)
			{
				if(result == MAV_RESULT_ACCEPTED)
					sbc_heartbeat.system_status = MAV_STATE_POWEROFF;
			}


		}
		case MAVLINK_MSG_ID_SET_MODE:
		{
			msg_target.system = mavlink_msg_set_mode_get_target_system(msg);
//			msg_target.component = mavlink_msg_set_mode_get_target_component(msg);

			if(comm_mavlink_check_target(&msg_target,msg))
			{
				MAV_MODE mode = mavlink_msg_set_mode_get_base_mode(msg);
				if(mode & MAV_MODE_FLAG_SAFETY_ARMED)
				{
					navigation_rxcmd_arm_disarm(1);
				}
				else
				{
					navigation_rxcmd_arm_disarm(0);
				}
			}
			break;
		}
		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
	//	{
	//		mavlink_msg_param_value_pack(mavlink_system.sysid, uint8_t component_id, mavlink_message_t* msg,
	//			       const char *param_id, float param_value, uint8_t param_type, uint16_t param_count, uint16_t param_index);
	//
			break;
	//	}
		case MAVLINK_MSG_ID_MISSION_ITEM:
		{
			msg_target.system = mavlink_msg_mission_item_get_target_system(msg);
			msg_target.component = mavlink_msg_mission_item_get_target_component(msg);

			if(comm_mavlink_check_target(&msg_target,msg))
			{
				mav_result = navigation_rx_mission_item(&msg_target, msg, &(answer_frame.mavlink_message));
			}
			break;
		}
		case MAVLINK_MSG_ID_MISSION_COUNT:
		{
			msg_target.system = mavlink_msg_mission_count_get_target_system(msg);
			msg_target.component = mavlink_msg_mission_count_get_target_component(msg);

			if(comm_mavlink_check_target(&msg_target,msg))
			{
				mav_result = navigation_rx_mission_items_start(&msg_target, msg, &(answer_frame.mavlink_message));
			}
			break;
		}
		case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
		{
			msg_target.system = mavlink_msg_mission_request_list_get_target_system(msg);
			msg_target.component = mavlink_msg_mission_request_list_get_target_component(msg);

			if(comm_mavlink_check_target(&msg_target,msg))
			{
				mav_result = navigation_tx_mission_items_start(&msg_target, msg, &(answer_frame.mavlink_message));
			}
			break;
		}
		case MAVLINK_MSG_ID_MISSION_REQUEST:
		{
			msg_target.system = mavlink_msg_mission_request_get_target_system(msg);
			msg_target.component = mavlink_msg_mission_request_get_target_component(msg);

			if(comm_mavlink_check_target(&msg_target,msg))
			{
				mav_result = navigation_tx_mission_item(&msg_target, msg, &(answer_frame.mavlink_message));
			}
			break;
		}
		case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
		{
			msg_target.system = mavlink_msg_mission_set_current_get_target_system(msg);
			msg_target.component = mavlink_msg_mission_set_current_get_target_component(msg);

			if(comm_mavlink_check_target(&msg_target,msg))
			{
				mav_result = navigation_next_mission_item(&msg_target, msg, &(answer_frame.mavlink_message));
			}
			break;
		}
		case MAVLINK_MSG_ID_ATTITUDE:
		{
			imu_update_attitude_from_mavlink(msg);
		}

		default:
			GPIO_toggle(Board_LED_RED);
			//Do nothing
			break;
	}


	// Answer, if required:
	if(mav_result == REPLY_TO_SENDER)
	{
		comm_mavlink_post_outbox(src_channel, &answer_frame);
	}
	else if(mav_result == FORWARD_MESSAGE)
	{
		COMM_CHANNEL fwd_channel;
		switch(msg_target.system){
			case SBC_SYSTEM_ID: fwd_channel = CHANNEL_APP_UART; break;
			case FBS_SYSTEM_ID: fwd_channel = CHANNEL_LORA; break;
			case APM_PLANNER_SYSTEM_ID: fwd_channel = CHANNEL_LORA; break;
		}

		comm_mavlink_post_outbox(fwd_channel, &answer_frame);
	}

}



void comm_init(){
	cli_init();

	/* MAVLINK GENERIC SETUP */

	mavlink_system.sysid = MAVLINK_SYSTEM_ID;                   ///< ID 25 for this rover
	mavlink_system.compid = MAV_COMP_ID_ALL;     ///< The component sending the message is all, it could be also a Linux process
	// Mavlink heartbeat
	// Define the system type, in this case a rover
	mavlink_heartbeat.type = MAV_TYPE_GROUND_ROVER;
	mavlink_heartbeat.autopilot = MAV_AUTOPILOT_GENERIC;
	mavlink_heartbeat.base_mode = MAV_MODE_MANUAL_DISARMED; ///< Booting up
	mavlink_heartbeat.system_status = MAV_STATE_STANDBY; ///< System ready for flight

	t_last_sbc_heartbeat = -2*SBC_HEARTBEAT_TIMEOUT;
	sbc_heartbeat.system_status = MAV_STATE_BOOT;
//	sbc_heartbeat.system_status = MAV_STATE_STANDBY; //TODO: remove, is just for testing purposes!!
	sbc_last_command_arm = -1;

}

void comm_task(){

	COMM_FRAME mail;

	comm_init();

	while(1){
		if(Mailbox_pend(comm_mailbox, &mail, BIOS_WAIT_FOREVER)){
			if((mail.direction) == CHANNEL_OUT)
			{
				comm_uart_send(mail.channel, &(mail.mavlink_message));
			}
			else
			{
				comm_mavlink_handler(mail.channel, &(mail.mavlink_message));
			}
		}
 	}

}
