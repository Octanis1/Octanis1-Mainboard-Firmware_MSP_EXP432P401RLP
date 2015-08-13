/*
 *  File: rockblock.h
 *  Description: Model for Iridium Satellite Modem "Rockblock"
 *  Author: Sam
 */

#ifndef __ROCKBLOCK_H
#define __ROCKBLOCK_H


int rockblock_open();

int rockblock_begin();

int rockblock_get_sleep_status();
int rockblock_get_net_availability();
int rockblock_get_signal_quality();

void rockblock_close();

//sends an SBD, then checks the inbox (checking costs 1 credit!)
int rockblock_send_receive_SBD(const uint8_t *tx_buffer, size_t tx_buffersize,
								uint8_t *rx_buffer, size_t *rx_buffersizePtr);

//gets an SBD message from rockblocks buffer
int rockblock_get_SBD_binary(uint8_t *rx_buffer, size_t *rx_buffersizePtr);


#endif
