#ifndef SHT2x_H
#define SHT2x_H
//==============================================================================
//    S E N S I R I O N   AG,  Laubisruetistr. 50, CH-8712 Staefa, Switzerland
//==============================================================================
// Project   :  SHT2x Sample Code (V1.2)
// File      :  SHT2x.h
// Author    :  MST
// Controller:  NEC V850/SG3 (uPD70F3740)
// Compiler  :  IAR compiler for V850 (3.50A)
// Brief     :  Sensor layer. Definitions of commands and registers,
//              functions for sensor access
//==============================================================================
//---------- Includes ----------------------------------------------------------
#include "SHT2xi2c.h"


//---------- Defines -----------------------------------------------------------

//#include "SHT2x_define.h"
//#include "bmp180.h"
//typedef float ft;



// sensor command
typedef enum{
  TRIG_T_MEASUREMENT_HM    = 0xE3, // command trig. temp meas. hold master
  TRIG_RH_MEASUREMENT_HM   = 0xE5, // command trig. humidity meas. hold master
  TRIG_T_MEASUREMENT_POLL  = 0xF3, // command trig. temp meas. no hold master
  TRIG_RH_MEASUREMENT_POLL = 0xF5, // command trig. humidity meas. no hold master
  USER_REG_W               = 0xE6, // command writing user register
  USER_REG_R               = 0xE7, // command reading user register
  SOFT_RESET               = 0xFE  // command soft reset
}etSHT2xCommand;

typedef enum {
  SHT2x_RES_12_14BIT       = 0x00, // RH=12bit, T=14bit
  SHT2x_RES_8_12BIT        = 0x01, // RH= 8bit, T=12bit
  SHT2x_RES_10_13BIT       = 0x80, // RH=10bit, T=13bit
  SHT2x_RES_11_11BIT       = 0x81, // RH=11bit, T=11bit
  SHT2x_RES_MASK           = 0x81  // Mask for res. bits (7,0) in user reg.
} etSHT2xResolution;

typedef enum {
  SHT2x_EOB_ON             = 0x40, // end of battery
  SHT2x_EOB_MASK           = 0x40, // Mask for EOB bit(6) in user reg.
} etSHT2xEob;

typedef enum {
  SHT2x_HEATER_ON          = 0x04, // heater on
  SHT2x_HEATER_OFF         = 0x00, // heater off
  SHT2x_HEATER_MASK        = 0x04, // Mask for Heater bit(2) in user reg.
} etSHT2xHeater;

// measurement signal selection
typedef enum{
  HUMIDITY,
  TEMPERATURE
}etSHT2xMeasureType;

typedef enum{
  I2C_ADR_W                = 128,   // sensor I2C address + write bit
  I2C_ADR_R                = 129    // sensor I2C address + read bit
}etI2cHeader;

#define SHT2x_WRITE_SIZE 1
#define SHT2X_READ_SIZE 3

#define CHECKSUM_ERROR 0
#define TIME_OUT_ERROR 0


#define SHT2x_INIT_VALUE ((u8)0)


//==============================================================================
unsigned char SHT2x_CheckCrc(unsigned char data[], unsigned char nbrOfBytes, unsigned char checksum);
//==============================================================================
// calculates checksum for n bytes of data and compares it with expected
// checksum
// input:  data[]       checksum is built based on this data
//         nbrOfBytes   checksum is built for n bytes of data
//         checksum     expected checksum
// return: error:       CHECKSUM_ERROR = checksum does not match
//                      0              = checksum matches

//==============================================================================
unsigned char SHT2x_ReadUserRegister(unsigned char *pRegisterValue);
//==============================================================================
// reads the SHT2x user register (8bit)
// input : -
// output: *pRegisterValue
// return: error
// ATTENTION: Not implemented (yet) for ti rtos. Should not be hard nor long to do however

//==============================================================================
unsigned char SHT2x_WriteUserRegister(unsigned char *pRegisterValue);
//==============================================================================
// writes the SHT2x user register (8bit)
// input : *pRegisterValue
// output: -
// return: error
// ATTENTION: Not implemented (yet) for ti rtos. Should not be hard nor long to do however

//==============================================================================
char SHT2x_Measure(etSHT2xMeasureType eSHT2xMeasureType, unsigned char *pMeasurand);
//==============================================================================
// measures humidity or temperature. This function polls every 10ms until
// measurement is ready.
// input:  eSHT2xMeasureType
// output: *pMeasurand:  humidity / temperature as raw value
// return: error
// note:   timing for timeout may be changed

//==============================================================================
unsigned char SHT2x_MeasureHM(etSHT2xMeasureType eSHT2xMeasureType, unsigned char *pMeasurand);
//==============================================================================
// measures humidity or temperature. This function waits for a hold master until
// measurement is ready or a timeout occurred.
// input:  eSHT2xMeasureType
// output: *pMeasurand:  humidity / temperature as raw value
// return: error
// note: timing for timeout may be changed
// note: Unused

//==============================================================================
unsigned char SHT2x_SoftReset();
//==============================================================================
// performs a reset
// input:  -
// output: -
// return: error
// ATTENTION: Not implemented (yet) for ti rtos. Should not be hard nor long to do however

//==============================================================================
float SHT2x_CalcRH(int u16sRH);
//==============================================================================
// calculates the relative humidity
// input:  sRH: humidity raw value (16bit scaled)
// return: pHumidity relative humidity [%RH]

//==============================================================================
float SHT2x_CalcTemperatureC(int u16sT);
//==============================================================================
// calculates temperature
// input:  sT: temperature raw value (16bit scaled)
// return: temperature [�C]

//==============================================================================
unsigned char SHT2x_GetSerialNumber(unsigned char u8SerialNumber[]);
//==============================================================================
// gets serial number of SHT2x according application note "How To
// Read-Out the Serial Number"
// note:   readout of this function is not CRC checked
//
// input:  -
// output: u8SerialNumber: Array of 8 bytes (64Bits)
//         MSB                                         LSB
//         u8SerialNumber[7]             u8SerialNumber[0]
//         SNA_1 SNA_0 SNB_3 SNB_2 SNB_1 SNB_0 SNC_1 SNC_0
// return: error
// ATTENTION: Not implemented (yet) for ti rtos. Should not be hard nor long to do however

//==============================================================================
int SHT2x_GetInfo (unsigned char* pMeasurand);
//==============================================================================
//input: u8 table containing the result of the i2c communication.
//return: u16 number containing the measurment result, ready for the calc* function.

//==============================================================================
float sht2x_get_temp();
//==============================================================================
// "main" function giving the temperature.

//==============================================================================
float sht2x_get_humidity();
//==============================================================================
// "main" function giving the relative humidity.
#endif
