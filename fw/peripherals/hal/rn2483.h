/*
 *  File: rn2483.h
 *  Description: Model for Microchip LoRa modem
 *  Author: Sam
 */

#ifndef __RN2483_H
#define __RN2483_H

#include "../../lib/mavlink/mavlink_types.h"
#define LORA_FRAME_SIZE 		2*MAVLINK_MAX_PACKET_LEN


//#define CONFIG_MODE 1 //undefine for normal operation!
//#define CONFIG_HIGH_DUTY 1
//#define CONFIG_SWISSCOM 1
//TODO: program crashes after a while in config_mode.
//note: apply a reset (lora reset pin to gnd) before running this mode!

#ifdef CONFIG_MODE
int rn2483_config();
#endif

int rn2483_begin();
void rn2483_end();


void rn2483_close();

//sends an SBD, then checks the inbox (checking costs 1 credit!)
int rn2483_send_receive(char * tx_buffer, int tx_size);


#endif
