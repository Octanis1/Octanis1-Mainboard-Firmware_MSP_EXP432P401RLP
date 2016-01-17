/*
  AS3935.h - AS3935 Franklin Lightning Sensor™ IC by AMS library
  Copyright (c) 2012 Raivis Rengelis (raivis [at] rrkb.lv). All rights reserved.
  Modified by Raffael Tschui (raffael [at] octanis.org) for TI-RTOS I2C Support.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 3 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef AS3935_h
#define AS3935_h


#include <limits.h>

// register access macros - register address, bitmask
#define AS3935_AFE_GB			0x00, 0x3E
#define AS3935_PWD				0x00, 0x01
#define AS3935_NF_LEV			0x01, 0x70
#define AS3935_WDTH				0x01, 0x0F
#define AS3935_CL_STAT			0x02, 0x40
#define AS3935_MIN_NUM_LIGH		0x02, 0x30
#define AS3935_SREJ				0x02, 0x0F
#define AS3935_LCO_FDIV			0x03, 0xC0
#define AS3935_MASK_DIST			0x03, 0x20
#define AS3935_INT				0x03, 0x0F
#define AS3935_DISTANCE			0x07, 0x3F
#define AS3935_DISP_LCO			0x08, 0x80
#define AS3935_DISP_SRCO			0x08, 0x40
#define AS3935_DISP_TRCO			0x08, 0x20
#define AS3935_TUN_CAP			0x08, 0x0F
#define AS3935_PRESET_DEFAULT	0x3C	, 0x00	//register to write direct command 0x96 to reset all registers
#define AS3935_CALIB_RCO			0x3D	, 0x00	//register to write direct command 0x96 to calibrate automatically the internal RC Oscillators

// other constants
#define AS3935_AFE_INDOOR		0x12
#define AS3935_AFE_OUTDOOR		0x0E
#define AS3935_NO_MASK			0x00		//when no bitmask is needed
#define AS3935_DIRECT_COMMAND	0x96		//register content to write


void lightning_reset();
uint8_t lightning_calibrate();

//keep them invisible for the moment:

//void powerDown();
//void powerUp();
//int interruptSource();
//void disableDisturbers();
//void enableDisturbers();
//int getMinimumLightnings();
//int setMinimumLightnings(int minlightning);
//int lightningDistanceKm();
//void setIndoors();
//void setOutdoors();
//int getNoiseFloor();
//int setNoiseFloor(int noisefloor);
//int getSpikeRejection();
//int setSpikeRejection(int srej);
//int getWatchdogThreshold();
//int setWatchdogThreshold(int wdth);
//void clearStats();
//void tune(int tuneValue);



void as3935_ISR();


#endif
