/*
 * bmp280_support.h
 *
 *  Created on: 26 Jul 2016
 *      Author: quentin
 */
#ifndef FW_PERIPHERALS_HAL_SI1133_SUPPORT_H_
#define FW_PERIPHERALS_HAL_SI1133_SUPPORT_H_

typedef	signed char  s8;/**< used for signed 8bit */
typedef	signed short int s16;/**< used for signed 16bit */
typedef	signed int s32;/**< used for signed 32bit */
typedef	signed long long int s64;/**< used for signed 64bit */

/*unsigned integer types*/
typedef	unsigned char u8;/**< used for unsigned 8bit */
typedef	unsigned short int u16;/**< used for unsigned 16bit */
typedef	unsigned int u32;/**< used for unsigned 32bit */
typedef	unsigned long long int u64;/**< used for unsigned 64bit */

#define ERROR                     ((s8)-1)
#define	SUCCESS					  ((u8)0)

#define SI1133_API
/*Enable the macro BMP280_API to use this support file */
/*----------------------------------------------------------------------------*
*  The following functions are used for reading and writing of
*	sensor data using I2C or SPI communication
*----------------------------------------------------------------------------*/
#ifdef SI1133_API

/*	\Brief: The function is used as I2C bus read
*	\Return : Status of the I2C read
*	\param dev_addr : The device address of the sensor
*	\param reg_addr : Address of the first register, where data is going to be read
*	\param reg_data : This is the data read from the sensor, which is held in an array
*	\param cnt : The no of data to be read
*/
s8 SI1133_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
 /*	\Brief: The function is used as I2C bus write
 *	\Return : Status of the I2C write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, where data is to be written
 *	\param reg_data : It is a value held in the array,
 *		which is written in the register
 *	\param cnt : The no of bytes of data to be written
 */
s8 SI1133_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);

#endif
/********************End of I2C/SPI function declarations***********************/
/*	Brief : The delay routine
 *	\param : delay in ms
*/
void SI1133_delay_msek(u32 msek);

#endif /* FW_PERIPHERALS_HAL_SI1133_SUPPORT_H_ */
