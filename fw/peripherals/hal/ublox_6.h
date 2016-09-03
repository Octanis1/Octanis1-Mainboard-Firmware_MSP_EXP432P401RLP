#ifndef __UBLOX_6_H
#define __UBLOX_6_H

#define UBLOX_6_NMEABUFFER_SIZE 82

/*NMEA sentence|	maximum possible length (chars incl. CRLF)
x 	GLL			51
x	GGA 			82
x	VTG			40
x	RMC			75
x	GSA			67
x	GSV			60 (per single line!)
	ZDA			34
	PFST,FOM		19
	PFST,PPS		35
=========================
 */


void ublox_6_open();
void ublox_6_close();


char * ublox_6_read();


#endif
