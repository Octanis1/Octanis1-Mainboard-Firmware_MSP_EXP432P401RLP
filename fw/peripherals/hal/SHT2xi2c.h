#ifndef I2C_HAL_H
#define I2C_HAL_H
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
#include "system.h"

//---------- Defines -----------------------------------------------------------
//I2C ports
//The communication on SDA and SCL is done by switching pad direction
//For a low level on SCL or SDA, direction is set to output. For a high level on
//SCL or SDA, direction is set to input. (pull up resistor active)
#define SDA      PM3H_bit.no0 //SDA on I/O P38 defines direction (input=1/output=0)
#define SDA_CONF P3H_bit.no0  //SDA level on output direction
#define SCL      PM3H_bit.no1 //SCL on I/O P39 defines direction (input=1/output=0)
#define SCL_CONF P3H_bit.no1  //SCL level on output direction

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
void I2c_Init ();
//==============================================================================
//Initializes the ports for I2C interface

//==============================================================================
void I2c_StartCondition ();
//==============================================================================
// writes a start condition on I2C-bus
// input : -
// output: -
// return: -
// note  : timing (delay) may have to be changed for different microcontroller
//       _____
// SDA:       |_____
//       _______
// SCL :        |___

//==============================================================================
void I2c_StopCondition ();
//==============================================================================
// writes a stop condition on I2C-bus
// input : -
// output: -
// return: -
// note  : timing (delay) may have to be changed for different microcontroller
//              _____
// SDA:   _____|
//            _______
// SCL :  ___|

//===========================================================================
u8t I2c_WriteByte (u8t txByte);
//===========================================================================
// writes a byte to I2C-bus and checks acknowledge
// input:  txByte  transmit byte
// output: -
// return: error
// note: timing (delay) may have to be changed for different microcontroller

//===========================================================================
u8t I2c_ReadByte (etI2cAck ack);
//===========================================================================
// reads a byte on I2C-bus
// input:  rxByte  receive byte
// output: rxByte
// note: timing (delay) may have to be changed for different microcontroller

#endif