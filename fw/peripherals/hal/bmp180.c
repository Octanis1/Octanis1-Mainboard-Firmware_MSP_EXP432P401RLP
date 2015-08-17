/*
 *  File: bmp180.h
 *  Description: Model for BMP180
 *  Author: Sam
 */

#include "../../../Board.h"
#include "bmp180.h"
#define BMP180_CHIP_ID   (0xd0) /* chip ID - always 0x55 */

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


static I2C_Handle      handle = NULL;

static int16_t ac1 = 0, ac2 = 0, ac3 = 0, b1 = 0, b2 = 0, mb = 0, mc = 0, md = 0;
static uint16_t ac4 = 0, ac5 = 0, ac6 = 0;


uint8_t read8(uint8_t register_addr){

	uint8_t readBuffer; //16 bits to be read only
	uint16_t returnValue = 0;
	I2C_Transaction i2cTransaction;


	if(handle != NULL){

		i2cTransaction.writeBuf = &register_addr;
		i2cTransaction.writeCount = sizeof(register_addr);

		i2cTransaction.readBuf = &readBuffer;
		i2cTransaction.readCount = sizeof(readBuffer); //make sure we read only 2 bytes

		i2cTransaction.slaveAddress = Board_BMP180_I2CADDR;

		int ret = I2C_transfer(handle, &i2cTransaction);

		if (!ret) {
		    cli_printf("read8 error \n", 0);
		}else{
			returnValue = readBuffer;
		}

	}

	return returnValue;
}

uint16_t read16(uint8_t register_addr){

	uint8_t readBuffer[2]; //16 bits to be read only
	uint16_t returnValue = 0;
	I2C_Transaction i2cTransaction;

	if(handle != NULL){

		i2cTransaction.writeBuf = &register_addr;
		i2cTransaction.writeCount = sizeof(register_addr);

		i2cTransaction.readBuf = readBuffer;
		i2cTransaction.readCount = 2; //make sure we read only 2 bytes

		i2cTransaction.slaveAddress = Board_BMP180_I2CADDR;

		int ret = I2C_transfer(handle, &i2cTransaction);

		if (!ret) {
		    cli_printf("read16 error \n", 0);
		}else{
			returnValue = readBuffer[0];
			returnValue <<= 8;
			returnValue |= readBuffer[1];
		}

	}

	return returnValue;
}

void write8(uint8_t register_addr, uint8_t data) {
	uint8_t writeBuffer[2];
	I2C_Transaction i2cTransaction;

	writeBuffer[0] = register_addr;
	writeBuffer[1] = data;

	if(handle != NULL){

		i2cTransaction.writeBuf = writeBuffer;
		i2cTransaction.writeCount = 2;

		i2cTransaction.slaveAddress = Board_BMP180_I2CADDR;
		int ret = I2C_transfer(handle, &i2cTransaction);

		if (!ret) {
		    cli_printf("write8 error\n", 0);
		}
	}
}

int32_t computeB5(int32_t UT) {
  int32_t X1 = (UT - (int32_t)ac6) * ((int32_t)ac5) >> 15;
  int32_t X2 = ((int32_t)mc << 11) / (X1+(int32_t)md);
  return X1 + X2;
}

uint16_t read_raw_temp(void) {
	write8(BMP180_CONTROL, BMP180_READTEMPCMD);
	Task_sleep(5);

	uint16_t raw_temp = read16(BMP180_TEMPDATA);

	return raw_temp;
}

//I2C connection setup
int bmp180_begin(){
	I2C_Params      params;
	I2C_Params_init(&params);

	cli_printf("attempt i2c \n", 0);

	handle = I2C_open(Board_I2C0, &params);
	if (!handle) {
	    cli_printf("I2C did not open \n", 0);
	    return 0;
	}else{
		cli_printf("i2c open \n",0);
	}




	int cid =  read8(BMP180_CHIP_ID);
	cli_printf("cid %X \n",cid);

	Task_sleep(100);
	cli_printf("attempt cal data \n",0);

	//read calibration data
	if(ac1 == 0){
		ac1 = read16(BMP180_CAL_AC1);
		cli_printf("ac1 %d\n",ac1);

		ac2 = read16(BMP180_CAL_AC2);

		ac3 = read16(BMP180_CAL_AC3);

		ac4 = read16(BMP180_CAL_AC4);

		ac5 = read16(BMP180_CAL_AC5);

		ac6 = read16(BMP180_CAL_AC6);

		b1 = read16(BMP180_CAL_B1);

		b2 = read16(BMP180_CAL_B2);


		mb = read16(BMP180_CAL_MB);

		mc = read16(BMP180_CAL_MC);

		md = read16(BMP180_CAL_MD);

		cli_printf("md %d\n",md);

	}



	/*
	write8(BMP180_CONTROL, BMP180_READTEMPCMD);
	wait 4.5ms
	uint16_t raw_temp = read16(BMP180_TEMPDATA);*/


	I2C_Transaction i2cTransaction;
	uint8_t wb[2];
	wb[0] = BMP180_CONTROL;
	wb[1] = BMP180_TEMPDATA;

	i2cTransaction.readBuf = NULL;
	i2cTransaction.readCount = 0;
	i2cTransaction.writeBuf = wb;
	i2cTransaction.writeCount = 2;

	i2cTransaction.slaveAddress = Board_BMP180_I2CADDR;
	int ret = I2C_transfer(handle, &i2cTransaction);


	Task_sleep(5);
	uint16_t raw_temp = read16(BMP180_TEMPDATA);

	cli_printf("t %d\n",raw_temp);



	return 1;
}

//ends an I2C "connection" to device
/*
void end(){
	I2C_close(handle);
	handle=NULL;
}
*/

float bmp180_get_temp(){


	int UT, B5;     // following ds convention
	float temp = 999.0;


		GPIO_write(Board_LED1, 1);

/*		UT = read_raw_temp();
		B5 = computeB5(UT);
		temp = (B5+8) >> 4;
		temp /= 10;
*/
		GPIO_write(Board_LED1, 0);

		//end();


	return temp;
}
