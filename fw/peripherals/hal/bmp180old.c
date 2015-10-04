/*
 *  File: bmp180.h
 *  Description: Model for BMP180
 *  Author: Sam
 *  Port for TI RTOS from following library:
 *
 *  ***************************************************
	  This is a library for the Adafruit BMP085/BMP180 Barometric Pressure + Temp sensor

	  Designed specifically to work with the Adafruit BMP085 or BMP180 Breakout
	  ----> http://www.adafruit.com/products/391
	  ----> http://www.adafruit.com/products/1603

	  These displays use I2C to communicate, 2 pins are required to
	  interface
	  Adafruit invests time and resources providing this open source code,
	  please support Adafruit and open-source hardware by purchasing
	  products from Adafruit!

	  Written by Limor Fried/Ladyada for Adafruit Industries.
	  BSD license, all text above must be included in any redistribution
	 ****************************************************
 */

#include "../../../Board.h"
#include "bmp180.h"
#include "i2c_helper.h"


#define BMP180_CHIP_ID   (0xd0) /* chip ID - always 0x55 */
#define BMP180_ADDR		 Board_BMP180_I2CADDR

#define BMP180_CAL_AC1           0xAA  // R   Calibration data (16 bits)
#define BMP180_CAL_AC2           0xAC  // R   Calibration data (16 bits)
#define BMP180_CAL_AC3           0xAE  // R   Calibration data (16 bits)
#define BMP180_CAL_AC4           0xB0  // R   Calibration data (16 bits)
#define BMP180_CAL_AC5           0xB2  // R   Calibration data (16 bits)
#define BMP180_CAL_AC6           0xB4  // R   Calibration data (16 bits)
#define BMP180_CAL_B1            0xB6  // R   Calibration data (16 bits)
#define BMP180_CAL_B2            0xB8  // R   Calibration data (16 bits)
#define BMP180_CAL_MB            0xBA  // R   Calibration data (16 bits)
#define BMP180_CAL_MC            0xBC  // R   Calibration data (16 bits)
#define BMP180_CAL_MD            0xBE  // R   Calibration data (16 bits)

#define BMP180_ULTRALOWPOWER 0
#define BMP180_STANDARD      1
#define BMP180_HIGHRES       2
#define BMP180_ULTRAHIGHRES  3

#define BMP180_CONTROL           0xF4
#define BMP180_TEMPDATA          0xF6
#define BMP180_PRESSUREDATA      0xF6
#define BMP180_READTEMPCMD       0x2E
#define BMP180_READPRESSURECMD   0x34



static int16_t ac1 = 0, ac2, ac3, b1, b2, mb, mc, md;
static uint16_t ac4, ac5, ac6;
static uint8_t oversampling = BMP180_ULTRALOWPOWER;


int32_t computeB5(int32_t UT) {
	int32_t X1 = (UT - (int32_t)ac6) * ((int32_t)ac5) >> 15;
	int32_t X2 = ((int32_t)mc << 11) / (X1+(int32_t)md);
	return X1 + X2;
}

uint16_t read_raw_temp(I2C_Handle handle) {
	write8(BMP180_ADDR, handle, BMP180_CONTROL, BMP180_READTEMPCMD);
	Task_sleep(5);

	uint16_t raw_temp = read16(BMP180_ADDR, handle, BMP180_TEMPDATA);

	return raw_temp;
}


uint32_t read_raw_pressure(I2C_Handle handle) {
	uint32_t raw;

	write8(BMP180_ADDR, handle, BMP180_CONTROL, BMP180_READPRESSURECMD + (oversampling << 6));

	if (oversampling == BMP180_ULTRALOWPOWER){
		Task_sleep(26);
	}else{
		Task_sleep(26);
	}

	raw = read16(BMP180_ADDR, handle, BMP180_PRESSUREDATA);

	raw <<= 8;
	raw |= read8(BMP180_ADDR, handle, BMP180_PRESSUREDATA+2);
	raw >>= (8 - oversampling);

	return raw;
}

//I2C connection setup
int bmp180_begin(I2C_Handle handle){

	//read chip id and verify that it's BMP180
	int cid =  read8(BMP180_ADDR, handle, BMP180_CHIP_ID);
	if(cid != 0x55) return 0;

	//read calibration data
	if(ac1 == 0){
		ac1 = read16(BMP180_ADDR, handle, BMP180_CAL_AC1);
		ac2 = read16(BMP180_ADDR, handle, BMP180_CAL_AC2);
		ac3 = read16(BMP180_ADDR, handle, BMP180_CAL_AC3);
		ac4 = read16(BMP180_ADDR, handle, BMP180_CAL_AC4);
		ac5 = read16(BMP180_ADDR, handle, BMP180_CAL_AC5);
		ac6 = read16(BMP180_ADDR, handle, BMP180_CAL_AC6);
		b1 = read16(BMP180_ADDR, handle, BMP180_CAL_B1);
		b2 = read16(BMP180_ADDR, handle, BMP180_CAL_B2);
		mb = read16(BMP180_ADDR, handle, BMP180_CAL_MB);
		mc = read16(BMP180_ADDR, handle, BMP180_CAL_MC);
		md = read16(BMP180_ADDR, handle, BMP180_CAL_MD);
	}

	return 1;
}



float bmp180_get_temp(I2C_Handle handle){

	float temp;
	int32_t b5;
	int32_t raw_temp = read_raw_temp(handle);

	b5 = computeB5(raw_temp);
	temp = (b5+8) >> 4;
	temp /= 10;

	return temp;
}

float bmp180_get_pressure(I2C_Handle handle){
	int32_t UT, UP, B3, B5, B6, X1, X2, X3;
	uint32_t B4, B7;
	float p = 0;

	UT = read_raw_temp(handle);
	UP = read_raw_pressure(handle);

	cli_printf("rp %d \n", UP);

	B5 = computeB5(UT);

	// do pressure calcs
	B6 = B5 - 4000;
	X1 = ((int32_t)b2 * ( (B6 * B6)>>12 )) >> 11;
	X2 = ((int32_t)ac2 * B6) >> 11;
	X3 = X1 + X2;
	B3 = ((((int32_t)ac1*4 + X3) << oversampling) + 2) / 4;

	X1 = ((int32_t)ac3 * B6) >> 13;
	X2 = ((int32_t)b1 * ((B6 * B6) >> 12)) >> 16;
	X3 = ((X1 + X2) + 2) >> 2;
	B4 = ((uint32_t)ac4 * (uint32_t)(X3 + 32768)) >> 15;
	B7 = ((uint32_t)UP - B3) * (uint32_t)( 50000UL >> oversampling );


	/*

	if (B7 < 0x80000000) {
		p = (float)((B7 * 2) / B4);

		//p=1;
		cli_printf("s B7 %X \n", p);


	}else{
		cli_printf("b B7 %X \n", B7);
		//p = (B7 / B4) * 2;
	}

*/
/*
	X1 = (p >> 8) * (p >> 8);
	X1 = (X1 * 3038) >> 16;
	X2 = (-7357 * p) >> 16;

	p = p + ((X1 + X2 + (int32_t)3791)>>4);
*/

	return p;
}


