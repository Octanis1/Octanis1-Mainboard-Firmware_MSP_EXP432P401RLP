/*
 *  File: hm10.h
 *  Description: BLE modem driver
 *  Author: Sam Sulaimanov
 */


#ifndef HM10_H_
#define HM10_H_

int hm10_begin(); //returns 1 if modem responds with OK
void hm10_end();
void hm10_send(char * tx_buffer, int tx_size);

#endif
