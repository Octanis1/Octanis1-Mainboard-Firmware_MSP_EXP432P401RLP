/*
 * bmp280_support.h
 *
 *  Created on: 26 Jul 2016
 *      Author: raffael
 */

#ifndef FW_PERIPHERALS_HAL_BMP280_SUPPORT_H_
#define FW_PERIPHERALS_HAL_BMP280_SUPPORT_H_


#define BMP280_API
/*Enable the macro BMP280_API to use this support file */
/*----------------------------------------------------------------------------*
*  The following functions are used for reading and writing of
*	sensor data using I2C or SPI communication
*----------------------------------------------------------------------------*/
#ifdef BMP280_API
s32 bmp280_init(u16 update_period_ms);

/*	\Brief: The function is used as I2C bus read
 *	\Return : Status of the I2C read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, where data is going to be read
 *	\param reg_data : This is the data read from the sensor, which is held in an array
 *	\param cnt : The no of bytes of data to be read
 */
s8 BMP280_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
 /*	\Brief: The function is used as I2C bus write
 *	\Return : Status of the I2C write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, where data is to be written
 *	\param reg_data : It is a value held in the array,
 *		which is written in the register
 *	\param cnt : The no of bytes of data to be written
 */
s8 BMP280_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);

/*
 * \Brief: SPI/I2C init routine
*/
s8 BMP280_I2C_routine(void);
#endif
/********************End of I2C/SPI function declarations***********************/
/*	Brief : The delay routine
 *	\param : delay in ms
*/
void BMP280_delay_msek(u32 msek);
/* This function is an example for reading sensor data
 *	\param: None
 *	\return: communication result
 */
s32 bmp280_data_readout(s32* temp_s32, u32* press_u32);

#endif /* FW_PERIPHERALS_HAL_BMP280_SUPPORT_H_ */
