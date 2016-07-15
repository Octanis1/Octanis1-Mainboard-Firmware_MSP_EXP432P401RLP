/*
 * comm.c
 *
 *  Created on: 10.08.2015
 *      Author: Sam
 */

#include "../../Board.h"
#include "comm.h"
#include <string.h>

#include "navigation.h"

#define N_TX_SLOT_FLAG_INTS		MAV_COMPONENT_ENUM_END/32+1 // +1 to round up
int comm_tx_slot_flags[MAVLINK_COMM_NUM_BUFFERS][N_TX_SLOT_FLAG_INTS]; // 1-bit flags storing

mavlink_system_t mavlink_system;
static mavlink_heartbeat_t mavlink_heartbeat;

mavlink_heartbeat_t comm_get_mavlink_heartbeat()
{return mavlink_heartbeat;}

void comm_mavlink_post_outbox(COMM_CHANNEL channel, COMM_FRAME* frame); //post to mailbox for outgoing messages
void comm_clear_tx_flag(COMM_CHANNEL channel, int component_id);

/* PUBLIC */

int comm_check_tx_slots(MAV_COMPONENT component) //check if outgoing message can be sent for a given destination and component id
{
	int ch_iter = 0;
#ifdef MAVLINK_ON_UART0_ENABLED
	for(ch_iter = 0; ch_iter<MAVLINK_COMM_NUM_BUFFERS; ch_iter++)
#else
	for(ch_iter = 1; ch_iter<MAVLINK_COMM_NUM_BUFFERS; ch_iter++)
#endif
	{
		if(( (comm_tx_slot_flags[ch_iter][component/32] & (1 << (component%32) )) != 0 ))
		{
			return ch_iter;
		}
	}
	return COMM_CHANNEL_NONE;
}

void comm_mavlink_broadcast(COMM_FRAME* frame) //posts to mailbox for all available channel slots for a given component
{
	int open_slot;
	while((open_slot = comm_check_tx_slots((frame->mavlink_message).compid)) > COMM_CHANNEL_NONE)
	{
		comm_mavlink_post_outbox(open_slot, frame);
		comm_clear_tx_flag(open_slot, (frame->mavlink_message).compid);
	}
}

void comm_mavlink_post_inbox(COMM_CHANNEL channel, mavlink_message_t *message); //post to mailbox for incoming messages


void comm_set_all_tx_flags(COMM_CHANNEL channel)
{
	int i;
	for(i=0;i<N_TX_SLOT_FLAG_INTS;i++)
	{
		comm_tx_slot_flags[channel][i] = 0xFFFFFFFF;
	}
}

void comm_set_tx_flag(COMM_CHANNEL channel, int component_id)
{
	comm_tx_slot_flags[channel][component_id/32] |= 1 << (component_id%32);  // Set the bit at the k-th position in comm_tx_slot_flags
}

/* PRIVATE */

void comm_mavlink_post_outbox(COMM_CHANNEL channel, COMM_FRAME* frame) //post to mailbox for outgoing messages
{
	frame->channel = channel;
	frame->direction = CHANNEL_OUT;
	Mailbox_post(comm_mailbox, frame, BIOS_NO_WAIT);
}

void comm_clear_tx_flag(COMM_CHANNEL channel, int component_id)
{
	comm_tx_slot_flags[channel][component_id/32] &= ~(1 << (component_id%32));
}

void comm_send(COMM_CHANNEL channel, mavlink_message_t *msg){

	static uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	static uint16_t mavlink_msg_len;

	// Copy the message to the send buffer and send
	mavlink_msg_len = mavlink_msg_to_send_buffer(buf, msg);

	switch(channel)
	{
		case CHANNEL_APP_UART:
#ifdef MAVLINK_ON_UART0_ENABLED
			if(serial_write(cli_stdout, buf, mavlink_msg_len))
			{//successful TX
				comm_clear_tx_flag(CHANNEL_APP_UART, msg->compid);
			}
			break;
#endif
		case CHANNEL_LORA:
		case CHANNEL_ROCKBLOCK:
		case CHANNEL_GSM:
			   //sim800_send_http(msg)
		default:
			break;
	}

}



COMM_MAV_RESULT comm_process_command(COMM_MAV_MSG_TARGET*  msg_target, mavlink_message_t *msg, mavlink_message_t *answer_msg){
	//TODO
	uint16_t command = mavlink_msg_command_long_get_command(msg);
	MAV_RESULT result = MAV_RESULT_DENIED;
	switch (command)
	{
		case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN: //#246 --> shut down SBPC. forward command to olimex.
			break;
		case MAV_CMD_NAV_LAND: //#21 [Land]
			break;
		case MAV_CMD_NAV_TAKEOFF: //#22 [Start]
			break;
		case MAV_CMD_OVERRIDE_GOTO: //#252 Hold / continue the current action [halt]
			break;
		case MAV_CMD_COMPONENT_ARM_DISARM: //#400 Arms / Disarms a component --- param1: 1 to arm, 0 to disarm
			break;
		default:
			result = MAV_RESULT_UNSUPPORTED;
			break;
	}

	mavlink_msg_command_ack_pack(mavlink_system.sysid, msg_target->component, answer_msg,
			command, result);

	return REPLY_TO_SENDER;
}

COMM_MAV_RESULT comm_set_mode(COMM_MAV_MSG_TARGET*  msg_target, mavlink_message_t *msg, mavlink_message_t *answer_msg){
	MAV_MODE mode = mavlink_msg_set_mode_get_base_mode(msg);
	mavlink_heartbeat.base_mode = mode;

	if(mode >= MAV_MODE_MANUAL_ARMED)
		// TODO: turn on SBPC and wait for boot to complete before changing state (via EPS)
		// TODO: --> probably better to move mavlink_heartbeat variable to EPS module
		// TODO: during boot process give status messages over MAV_MSG_STATUSTEXT
		// TODO: add conditions that may stop from changing state to armed, for example low battery.
		mavlink_heartbeat.system_status = MAV_STATE_ACTIVE;

	else
		// TODO: shut down SBPC before changing state
		mavlink_heartbeat.system_status = MAV_STATE_STANDBY;

	return NO_ANSWER;
}



//returns true if message target is this system --> you can then continue decoding the message.
//otherwise it should forward the message to the corresponding target (i.e. EPS, SBC, etc...)
int comm_mavlink_check_target(COMM_MAV_MSG_TARGET* target, mavlink_message_t *msg)
{
	if(target->system == mavlink_system.sysid)
	{
		return 1;
	}
	else if(target->system == SBPC_SYSTEM_ID)
	{
		COMM_FRAME forward_frame;
		(forward_frame.mavlink_message) = (*msg);

		// forward to SBPC connected to UART0
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

	switch(msg->msgid){
		case MAVLINK_MSG_ID_HEARTBEAT:
		{
		// E.g. read GCS heartbeat and go into
		// comm lost mode if timer times out
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

		case MAVLINK_MSG_ID_SET_MODE:
		{
			msg_target.system = mavlink_msg_set_mode_get_target_system(msg);
//			msg_target.component = mavlink_msg_set_mode_get_target_component(msg);

			if(comm_mavlink_check_target(&msg_target,msg))
			{
				mav_result = comm_set_mode(&msg_target, msg, &(answer_frame.mavlink_message));
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

	uint64_t system_time = 1000 * 1000 * (1234); //TODO get time on each message packing
}

void comm_task(){

	COMM_FRAME mail;


	comm_init();

	while(1){
		if(Mailbox_pend(comm_mailbox, &mail, BIOS_WAIT_FOREVER)){
			if((mail.direction) == CHANNEL_OUT)
			{
				comm_send(mail.channel, &(mail.mavlink_message));
			}
			else
			{
				comm_mavlink_handler(mail.channel, &(mail.mavlink_message));
			}
		}
 	}

}
