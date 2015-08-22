/*
 * bno055.c
 *
 *  Created on: 19 Aug 2015
 *      Author: Sam
 */
#include "../../../Board.h"
#include "bno055.h"
#include "i2c_helper.h"


//Puts the chip in the specified operating mode
void bno055_set_mode(bno055_addr_t addr,  I2C_Handle handle, bno055_opmode_t mode)
{
  write8(addr, handle, BNO055_OPR_MODE_ADDR, mode);
  Task_sleep(30);
}

//initialise the chip
int bno055_begin(bno055_addr_t addr, I2C_Handle handle){

	/* Make sure we have the right device */
	uint8_t id = read8(addr, handle,  BNO055_CHIP_ID_ADDR);

	if(id != BNO055_CHIP_ID){
		Task_sleep(1000); // hold on for boot
		id = read8(addr, handle,  BNO055_CHIP_ID_ADDR);

		if(id != BNO055_CHIP_ID) {
		  return 0;  // still not? ok bail
		}
	}

	/* Switch to config mode (just in case since this is the default) */
	bno055_set_mode(addr, handle, OPERATION_MODE_CONFIG);

	/* Reset */
	write8(addr, handle, BNO055_SYS_TRIGGER_ADDR, 0x20);
	while (read8(addr, handle, BNO055_CHIP_ID_ADDR) != BNO055_CHIP_ID){
		Task_sleep(10);
	}
	Task_sleep(50);

	/* Set to normal power mode */
	write8(addr, handle, BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
	Task_sleep(10);

	write8(addr, handle, BNO055_PAGE_ID_ADDR, 0);

	/* Set the output units */
	/*
	uint8_t unitsel = (0 << 7) | // Orientation = Android
					(0 << 4) | // Temperature = Celsius
					(0 << 2) | // Euler = Degrees
					(1 << 1) | // Gyro = Rads
					(0 << 0);  // Accelerometer = m/s^2
	write8(BNO055_UNIT_SEL_ADDR, unitsel);
	*/

	write8(addr, handle, BNO055_SYS_TRIGGER_ADDR, 0x0);
	Task_sleep(10);
	/* Set the requested operating mode (see section 3.3) */
	bno055_set_mode(addr, handle, OPERATION_MODE_CONFIG); //??
	Task_sleep(20);

	return 1;

}
