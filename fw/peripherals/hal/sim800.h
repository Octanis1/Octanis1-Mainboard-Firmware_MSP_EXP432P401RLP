/*
 *  File: sim800.h
 *  Description: Model for vhf radio HX1 to do FSK
 *  Author: Sam Sulaimanov
 */


#ifndef SIM800_H_
#define SIM800_H_

int sim800_begin(); //returns 1 if modem responds with OK
void sim800_end();

void sim800_send_http(char * tx_buffer, int tx_size);
const char* sim800_get_battery_voltage();

#endif
