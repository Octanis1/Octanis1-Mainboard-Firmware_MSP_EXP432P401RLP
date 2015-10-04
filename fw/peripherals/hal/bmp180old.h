/*
 *  File: bmp180.h
 *  Description: Model for BMP180
 *  Author: Sam
 */



#ifndef __BMP180_H
#define __BMP180_H

float bmp180_get_pressure(I2C_Handle handle);
float bmp180_get_temp(I2C_Handle handle);
int bmp180_begin(I2C_Handle handle);

#endif
