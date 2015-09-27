/*
 * i2c_helper.h
 *
 *  Created on: 19 Aug 2015
 *      Author: Sam
 */

#ifndef FW_PERIPHERALS_HAL_I2C_HELPER_H_
#define FW_PERIPHERALS_HAL_I2C_HELPER_H_



uint8_t read8(unsigned char i2c_addr, I2C_Handle handle, uint8_t register_addr);
uint16_t read16(unsigned char i2c_addr, I2C_Handle handle, uint8_t register_addr);
void write8(unsigned char i2c_addr, I2C_Handle handle, uint8_t register_addr, uint8_t data);

extern I2C_Handle i2c_helper_handle;

I2C_Handle i2c_helper_get_handle();

void i2c_helper_init_handle();

#endif /* FW_PERIPHERALS_HAL_I2C_HELPER_H_ */
