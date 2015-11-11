#ifndef SHT2x_I2C_HAL_H
#define SHT2x_I2C_HAL_H
//==============================================================================
//    S E N S I R I O N   AG,  Laubisruetistr. 50, CH-8712 Staefa, Switzerland
//==============================================================================
// Project   :  SHT2x Sample Code (V1.2)
// File      :  I2C_HAL.h
// Author    :  MST
// Controller:  NEC V850/SG3 (uPD70F3740)
// Compiler  :  IAR compiler for V850 (3.50A)
// Brief     :  I2C Hardware abstraction layer
//==============================================================================

//---------- Includes ----------------------------------------------------------
//#include "SHT2x_define.h"
//#include "bmp180.h"

//---------- Defines -----------------------------------------------------------
#define SHT2x_I2C_ERROR 0
//typedef float ft;

//---------- Enumerations ------------------------------------------------------
//  I2C level
typedef enum{
  LOW                      = 0,
  HIGH                     = 1,
}etI2cLevel;

// I2C acknowledge
typedef enum{
  ACK                      = 0,
  NO_ACK                   = 1,
}etI2cAck;

//==============================================================================
char SHT2x_I2C_write(unsigned char dev_addr, unsigned char *reg_data, unsigned char cnt);
//==============================================================================

//==============================================================================
char SHT2x_I2C_read(unsigned char dev_addr, unsigned char *reg_data, unsigned char cnt);
//==============================================================================
//Write and read function


//===========================================================================
void SHT2x_delay_msek(int msek);
//===========================================================================
// Put the tasl on sleep for a while

#endif
