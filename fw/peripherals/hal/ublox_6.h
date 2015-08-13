


#ifndef __UBLOX_6_H
#define __UBLOX_6_H


#define UBLOX_6_NMEABUFFER_SIZE 300


int ublox_6_open();
void ublox_6_close();


char * ublox_6_read();


#endif
