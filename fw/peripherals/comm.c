/*
 * comm.c
 *
 *  Created on: 10.08.2015
 *      Author: Sam
 */

#include "../../Board.h"
#include "comm.h"
#include <string.h>

#define N_TX_SLOT_FLAG_INTS		MAV_COMPONENT_ENUM_END/32+1 // +1 to round up
int comm_tx_slot_flags[MAVLINK_COMM_NUM_BUFFERS][N_TX_SLOT_FLAG_INTS]; // 1-bit flags storing

mavlink_system_t mavlink_system;

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
			if(serial_write(stdout, buf, mavlink_msg_len))
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



void comm_mavlink_handler(COMM_CHANNEL src_channel, mavlink_message_t *msg){
	COMM_MAV_MSG_TARGET target;

	static uint8_t mission_count;
	static uint16_t mission_item_id;

	switch(msg->msgid){
	  case MAVLINK_MSG_ID_HEARTBEAT:
	        {
		    // E.g. read GCS heartbeat and go into
            // comm lost mode if timer times out
	        }
	        break;
	case MAVLINK_MSG_ID_COMMAND_LONG:
		// EXECUTE ACTION
		break;

	case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT:
			GPIO_toggle(Board_LED_RED);
		break;

	case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
//	{
//		mavlink_msg_param_value_pack(mavlink_system.sysid, uint8_t component_id, mavlink_message_t* msg,
//			       const char *param_id, float param_value, uint8_t param_type, uint16_t param_count, uint16_t param_index);
//
		break;
//	}

	case MAVLINK_MSG_ID_MISSION_ITEM:
	{

		//TODO: decode message with mavlink_msg_mission_item_decode() and save mission_item

		uint8_t sysid = mavlink_msg_mission_item_get_target_system(msg);
		uint8_t target_cmp = mavlink_msg_mission_item_get_target_component(msg);

		if(mission_count > mission_item_id)
		{
		static COMM_FRAME frame;
		mavlink_msg_mission_request_pack(mavlink_system.sysid, target_cmp, &(frame.mavlink_message),
				msg->sysid, target_cmp, mission_item_id);

		mission_item_id++;
		comm_mavlink_post_outbox(src_channel, &frame);
		}
		else
		{
			static COMM_FRAME frame;
			mavlink_msg_mission_ack_pack(mavlink_system.sysid, target_cmp, &(frame.mavlink_message),
					msg->sysid, target_cmp, 	MAV_MISSION_ACCEPTED);

			comm_mavlink_post_outbox(src_channel, &frame);
		}
		break;
	}


	case MAVLINK_MSG_ID_MISSION_COUNT:
	{
		uint8_t sysid = mavlink_msg_mission_count_get_target_system(msg);
		uint8_t target_cmp = mavlink_msg_mission_count_get_target_component(msg);
		mission_count = mavlink_msg_mission_count_get_count(msg);
		mission_item_id = 0;

		/* send mission acknowledge */
		// Initialize the message buffer

		static COMM_FRAME frame;
		mavlink_msg_mission_request_pack(mavlink_system.sysid, target_cmp, &(frame.mavlink_message),
				msg->sysid, target_cmp, 0);

		comm_mavlink_post_outbox(src_channel, &frame);

		break;
	}
	case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
	{
		uint8_t sysid = mavlink_msg_mission_request_list_get_target_system(msg);
		uint8_t target_cmp =mavlink_msg_mission_request_list_get_target_component(msg);
		mavlink_mission_request_list_t* mission_request_list;

		/* Acknowledge by sending the mission count */
		static COMM_FRAME frame;
		mavlink_msg_mission_count_pack(mavlink_system.sysid, target_cmp,  &(frame.mavlink_message),
				msg->sysid, target_cmp, mission_count);

		comm_mavlink_post_outbox(src_channel, &frame);

		break;
	}
	case MAVLINK_MSG_ID_MISSION_REQUEST:
	{


		break;
	}
	default:
		GPIO_toggle(Board_LED_RED);
		//Do nothing
		break;
	}

}



void comm_init(){
	cli_init();

	/* MAVLINK GENERIC SETUP */

	mavlink_system.sysid = MAVLINK_SYSTEM_ID;                   ///< ID 25 for this rover
	mavlink_system.compid = MAV_COMP_ID_ALL;     ///< The component sending the message is all, it could be also a Linux process

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
