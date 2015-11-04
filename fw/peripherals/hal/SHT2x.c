//==============================================================================
//    S E N S I R I O N   AG,  Laubisruetistr. 50, CH-8712 Staefa, Switzerland
//==============================================================================
// Project   :  SHT2x Sample Code (V1.2)
// File      :  SHT2x.c
// Author    :  MST
// Controller:  NEC V850/SG3 (uPD70F3740)
// Compiler  :  IAR compiler for V850 (3.50A)
// Brief     :  Sensor layer. Functions for sensor access
//==============================================================================

//---------- Includes ----------------------------------------------------------
#include "SHT2x.h"
#include "../../../Board.h"

//==============================================================================
float sht2x_get_temp()
//==============================================================================
{
	u8 pMeasurand[3]; //table where the result of the communication is put
	u16 data =0;
	float result =0;
	s8 error =1;

	error = SHT2x_Measure(TEMP, pMeasurand);

	data = SHT2x_GetInfo (pMeasurand);

	result = SHT2x_CalcTemperatureC(data);

	return result;
}

//==============================================================================
float sht2x_get_humidity()
//==============================================================================
{
	u8 pMeasurand[3]; //table where the result of the communication is put
	u16 data =0;
	float result =0;
	s8 error =1;

	error = SHT2x_Measure(HUMIDITY, pMeasurand);

	data = SHT2x_GetInfo (pMeasurand);

	result = SHT2x_CalcRH(data);

	return result;
}

//==============================================================================
u8 SHT2x_CheckCrc(u8 data[], u8 nbrOfBytes, u8 checksum)
//==============================================================================
{
  u8 crc = 0;
  u8 byteCtr;
  //calculates 8-Bit checksum with given polynomial
  for (byteCtr = 0; byteCtr < nbrOfBytes; ++byteCtr)
  { crc ^= (data[byteCtr]);
    for (u8 bit = 8; bit > 0; --bit)
    { if (crc & 0x80) crc = (crc << 1) ^ POLYNOMIAL;
      else crc = (crc << 1);
    }
  }
  if (crc != checksum) return CHECKSUM_ERROR;
  else return 0;
}

//===========================================================================
u8 SHT2x_ReadUserRegister(u8 *pRegisterValue)
//===========================================================================
{
/*  u8 checksum;   //variable for checksum byte
  u8 error=0;    //variable for error code

  I2c_StartCondition();
  error |= I2c_WriteByte (I2C_ADR_W);
  error |= I2c_WriteByte (USER_REG_R);
  I2c_StartCondition();
  error |= I2c_WriteByte (I2C_ADR_R);
  *pRegisterValue = I2c_ReadByte(ACK);
  checksum=I2c_ReadByte(NO_ACK);
  error |= SHT2x_CheckCrc (pRegisterValue,1,checksum);
  I2c_StopCondition();
  return error;*/
	return 0;
}

//===========================================================================
u8 SHT2x_WriteUserRegister(u8 *pRegisterValue)
//===========================================================================
{
  /*u8 error=0;   //variable for error code

  I2c_StartCondition();
  error |= I2c_WriteByte (I2C_ADR_W);
  error |= I2c_WriteByte (USER_REG_W);
  error |= I2c_WriteByte (*pRegisterValue);
  I2c_StopCondition();
  return error;*/
	return 0;
}


//===========================================================================
s8 SHT2x_Measure(etSHT2xMeasureType eSHT2xMeasureType, u8 *pMeasurand)
//===========================================================================
{

  s8  error=1;    //error variable
  u16 i=0;        //counting variable
  u8 write_buffer =0;

  //-- write I2C sensor address and command --

  switch(eSHT2xMeasureType)
  { case HUMIDITY: write_buffer = TRIG_RH_MEASUREMENT_POLL; break;
    case TEMP    : write_buffer = TRIG_T_MEASUREMENT_POLL;  break;
    default: ; //error message?
  }

  error = SHT2x_I2C_write (Board_SHT21_I2CADDR, &write_buffer, 1);

  //-- poll every 10ms for measurement ready. Timeout after 20 retries (200ms)--
  do
  { SHT2x_delay_msek(10);  //delay 10ms
    if(i++ >= 20) break;
  } while(!SHT2x_I2C_read(Board_SHT21_I2CADDR, pMeasurand, 3));
  if (i>=20) error |= TIME_OUT_ERROR;


  //-- verify checksum --
  //The 2 first bytes recieved are the data measured and the last one is a
  //checksum we use to verify the validity of the data.
  error |= SHT2x_CheckCrc (pMeasurand, 2, pMeasurand[2]);

  return error;
}

//===========================================================================
u8 SHT2x_SoftReset()
//===========================================================================
{
  u8  error=0;           //error variable

  //I2c_StartCondition();
  //error |= I2c_WriteByte (I2C_ADR_W); // I2C Adr
  //error |= I2c_WriteByte (SOFT_RESET);                            // Command
  //I2c_StopCondition();

  //DelayMicroSeconds(15000); // wait till sensor has restarted

  return error;
}

//==============================================================================
float SHT2x_CalcRH(u16 u16sRH)
//==============================================================================
{
  ft humidityRH;              // variable for result

  u16sRH &= ~0x0003;          // clear bits [1..0] (status bits)
  //-- calculate relative humidity [%RH] --

  humidityRH = -6.0 + 125.0/65536 * (ft)u16sRH; // RH= -6 + 125 * SRH/2^16
  return humidityRH;
}

//==============================================================================
float SHT2x_CalcTemperatureC(u16 u16sT)
//==============================================================================
{
  ft temperatureC;            // variable for result

  u16sT &= ~0x0003;           // clear bits [1..0] (status bits)

  //-- calculate temperature [�C] --
  temperatureC= -46.85 + 175.72/65536 *(ft)u16sT; //T= -46.85 + 175.72 * ST/2^16
  return temperatureC;
}

//==============================================================================
u8 SHT2x_GetSerialNumber(u8 u8SerialNumber[])
//==============================================================================
{
  u8  error=0;                          //error variable

  //Read from memory location 1
  /*I2c_StartCondition();
  error |= I2c_WriteByte (I2C_ADR_W);    //I2C address
  error |= I2c_WriteByte (0xFA);         //Command for readout on-chip memory
  error |= I2c_WriteByte (0x0F);         //on-chip memory address
  I2c_StartCondition();
  error |= I2c_WriteByte (I2C_ADR_R);    //I2C address
  u8SerialNumber[5] = I2c_ReadByte(ACK); //Read SNB_3
  I2c_ReadByte(ACK);                     //Read CRC SNB_3 (CRC is not analyzed)
  u8SerialNumber[4] = I2c_ReadByte(ACK); //Read SNB_2
  I2c_ReadByte(ACK);                     //Read CRC SNB_2 (CRC is not analyzed)
  u8SerialNumber[3] = I2c_ReadByte(ACK); //Read SNB_1
  I2c_ReadByte(ACK);                     //Read CRC SNB_1 (CRC is not analyzed)
  u8SerialNumber[2] = I2c_ReadByte(ACK); //Read SNB_0
  I2c_ReadByte(NO_ACK);                  //Read CRC SNB_0 (CRC is not analyzed)
  I2c_StopCondition();

  //Read from memory location 2
  I2c_StartCondition();
  error |= I2c_WriteByte (I2C_ADR_W);    //I2C address
  error |= I2c_WriteByte (0xFC);         //Command for readout on-chip memory
  error |= I2c_WriteByte (0xC9);         //on-chip memory address
  I2c_StartCondition();
  error |= I2c_WriteByte (I2C_ADR_R);    //I2C address
  u8SerialNumber[1] = I2c_ReadByte(ACK); //Read SNC_1
  u8SerialNumber[0] = I2c_ReadByte(ACK); //Read SNC_0
  I2c_ReadByte(ACK);                     //Read CRC SNC0/1 (CRC is not analyzed)
  u8SerialNumber[7] = I2c_ReadByte(ACK); //Read SNA_1
  u8SerialNumber[6] = I2c_ReadByte(ACK); //Read SNA_0
  I2c_ReadByte(NO_ACK);                  //Read CRC SNA0/1 (CRC is not analyzed)
  I2c_StopCondition();*/

  return error;
}


//==============================================================================
u16 SHT2x_GetInfo (u8* pMeasurand)
//==============================================================================
//The RH or T value is stoked on 2 entries in a u8 table. Here we put them in a u16
{
	u16 info = 0;
	info = (u16)pMeasurand[0];
	info = info << 8;
	info += (u16)pMeasurand[1];
	return info;
}
