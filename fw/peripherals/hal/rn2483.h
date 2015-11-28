/*
 *  File: rn2483.h
 *  Description: Model for Iridium Satellite Modem "Rockblock"
 *  Author: Sam
 */

#ifndef __RN2483_H
#define __RN2483_H

// #define CONFIG_MODE 1 //undefine for normal operation!
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
