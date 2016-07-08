/*
 * comm.c
 *
 *  Created on: 10.08.2015
 *      Author: Sam
 */

#include "../../Board.h"
#include "comm.h"
#include <string.h>
#include "hal/rn2483.h"

#include "navigation.h"

#define LORA_FRAME_SIZE 		2*MAVLINK_MAX_PACKET_LEN

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


#include "../lib/printf.h"
void comm_send_string_over_lora(uint8_t* txdata, uint16_t stringlength)
{
	/** Prepare hex string for LoRa **/
	char hex_string_byte[2];
	char hex_string[LORA_FRAME_SIZE]; //TODO: ATTENTION: this is too small! need to change this
	memset(&hex_string, 0, sizeof(hex_string));

	int i;
	for(i=0; i<stringlength; i++){
		memset(&hex_string_byte, 0, sizeof(hex_string_byte));
		tfp_sprintf(hex_string_byte, "%02x", txdata[i]);
		strcat(hex_string, hex_string_byte);
	}
	/** end hex string **/

	rn2483_send_receive(hex_string, 2*stringlength);
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
#endif
			break;
		case CHANNEL_LORA:
#ifdef LORA_ENABLED
			comm_send_string_over_lora(buf, mavlink_msg_len); //FIXME: this is just test code
#endif
			break;
		case CHANNEL_ROCKBLOCK:
		case CHANNEL_GSM:
			   //sim800_send_http(msg)
		default:
			break;
	}

}

//returns true if message target is this system --> you can then continue decoding the message.
//otherwise it should forward the message to the corresponding target (i.e. EPS, SBC, etc...)
int comm_mavlink_check_target(COMM_MAV_MSG_TARGET* target, mavlink_message_t *msg)
{
	if(target->system == mavlink_system.sysid)
	{
		return 1;
	}
	else{
		//TODO: forward the message to corresponding system
		return 0;
	}
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

#ifdef LORA_ENABLED
	#ifdef CONFIG_MODE
		int comm_result=rn2483_config();
		if(comm_result)
			serial_printf(cli_stdout,"LoRa config failed: %d", comm_result);
		else
			serial_printf(cli_stdout,"LoRa config success: %d", comm_result);
	#else
		rn2483_begin();
	#endif
#endif

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
