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
	for(ch_iter = 0; ch_iter<MAVLINK_COMM_NUM_BUFFERS; ch_iter++)
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

	switch(channel) {
#ifdef MAVLINK_ON_UART0_ENABLED
	   case CHANNEL_APP_UART:
		  {

		  }
 	      break;
#endif

	   case CHANNEL_GSM:
		   //sim800_send_http(msg)
 	      break;

	   default: break;

 	}

}



void comm_mavlink_handler(mavlink_message_t *msg){

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
	default:
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

	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t mavlink_msg_len;

	comm_init();

	while(1){
		if(Mailbox_pend(comm_mailbox, &mail, BIOS_WAIT_FOREVER)){
			if((mail.direction) == CHANNEL_OUT)
			{
				// Copy the message to the send buffer and send
				mavlink_msg_len = mavlink_msg_to_send_buffer(buf, &(mail.mavlink_message));

				switch(mail.channel)
				{
					case CHANNEL_APP_UART:
						if(serial_write(stdout, buf, mavlink_msg_len))
						{//successful TX
							comm_clear_tx_flag(CHANNEL_APP_UART, mail.mavlink_message.compid);
						}

						break;
					case CHANNEL_LORA:
					case CHANNEL_ROCKBLOCK:
					case CHANNEL_GSM:
					default:
						break;
				}

			}
			else
			{
				//  comm_mavlink_handler()

			}
		}
 	}

}
