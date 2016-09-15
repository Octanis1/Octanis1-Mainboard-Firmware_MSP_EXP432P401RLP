/*
 *  File: rockblock.h
 *  Description: Model for Iridium Satellite Modem "Rockblock"
 *  Author: Sam
 */

#ifndef __ROCKBLOCK_H
#define __ROCKBLOCK_H

#define ROCKBLOCK_MESSAGE_SIZE (100)


int rockblock_open();

int rockblock_begin();

int rockblock_get_sleep_status();
int rockblock_get_net_availability();
int rockblock_get_signal_quality();
int rockblock_get_health();
int rockblock_get_tx_buffer_fill();
int rockblock_get_remaining_msg();
//empties the rockblock buffer and initiates a send/receive action
int rockblock_flush();

void rockblock_close();

/*
//sends an SBD, then checks the inbox (checking costs 1 credit!)
int rockblock_send_receive_SBD(const uint8_t *tx_buffer, size_t tx_buffersize,
								uint8_t *rx_buffer, size_t *rx_buffersizePtr);*/

//adds an SBD message to the rockblock's buffer. Automatic flush only if buffer full
int rockblock_add_SBD_binary(uint8_t *tx_buffer, uint16_t *tx_buffersizePtr);

struct rockblock_msg {
	char *msg;
	struct rockblock_msg *next;
};


#endif
