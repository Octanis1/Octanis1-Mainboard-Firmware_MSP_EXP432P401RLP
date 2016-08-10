#include "SI1133.h"
/***************************************************************
 * I2C utility functions for si1133 uv - vis - ir light sensor
 * read/write use the bus-read bus-write functions provided
 * for Bosch sensors (eg bme280 / bmp280)
 **************************************************************/

//read 1 from register reg and store it to the u8 array reg_data
//return ERROR or SUCCESS
static uint8_t si1133_read8(uint8_t reg, u8 *reg_data) {
	return SI1133_I2C_bus_read(SI1133_ADDR, reg, reg_data, 1);
}

//read 2 from register reg and store it to the u8 array reg_data
//return ERROR or SUCCESS..
static uint16_t si1133_read16(uint8_t reg, u8 *reg_data) {
	return SI1133_I2C_bus_read(SI1133_ADDR, reg, reg_data, 2);
}

//write 1 byte val to register reg
static void si1133_write8(uint8_t reg, uint8_t val) {
  SI1133_I2C_bus_write(SI1133_ADDR, reg, &val, 1);
}

/*********************************************************************/
static uint8_t writeParam(uint8_t p, uint8_t v, u8 *reg_data) {
  si1133_write8(SI1133_REG_HOSTIN0, v);
  si1133_write8(SI1133_REG_COMMAND, p | SI1133_PARAM_SET);
  return si1133_read8(SI1133_REG_RESPONSE1, reg_data);
}

static uint8_t readParam(uint8_t p, u8 *reg_data ) {
  si1133_write8(SI1133_REG_COMMAND, p | SI1133_PARAM_QUERY);
  return si1133_read8(SI1133_REG_RESPONSE1, reg_data);
}

/***************************************************************
 * Fonction to be called to initialize the Si1133 sensor
 **************************************************************/
bool si1133_begin(void) {
  //i2c_helper_init_handle(); not used as initialized in weather.c
  uint8_t id;
  u8 response; //can be used to access the status after writing a parameter
  si1133_read8(SI1133_REG_PARTID, &id);
  if (id != 0x33) return false; // mira si es si1133
  //device reset at init
  si1133_reset();
  writeParam(SI1133_PARAM_MEASRATEH,0,&response);
  writeParam(SI1133_PARAM_MEASRATEL,1,&response);
  writeParam(SI1133_PARAM_MEASCOUNT0,5,&response);
  writeParam(SI1133_PARAM_MEASCOUNT1,10,&response);
  //seleccionamos los canales 0(16bits) y 1(24bits)
  //por lo que los resultados estaran en
  //HOTSOUT[0-1]------> canal 0 , 2 registros por la resolucion de 16bits
  //HOTSOUT[2-2]------> canal 1, etc
  writeParam(SI1133_PARAM_CHLIST,(uint8_t)0X0F,&response);
  //=======================================================
  //configuraciones para el canal 0
  //seleccionamos el rate y el photodiodo
  writeParam(SI1133_PARAM_ADCCONFIG0,RATE_NORMAL| F_UV, &response ); //reading UV
  writeParam(SI1133_PARAM_ADCSENS0,0x60, &response);      //0x60 correspond to 64 data acquisition summed together
  writeParam(SI1133_PARAM_ADCPSOT0,BITS_16, &response);   //resolution
  writeParam(SI1133_PARAM_MEASCONFIG0,COUNT0, &response);
  //=======================================================
  writeParam(SI1133_PARAM_ADCCONFIG1,RATE_NORMAL| F_LARGE_IR, &response );
  writeParam(SI1133_PARAM_ADCSENS1,0x80, &response); //0x80 corresponds to low ADC gain (/14.5) for outdoor measurments
  writeParam(SI1133_PARAM_ADCPSOT1,BITS_16, &response);
  writeParam(SI1133_PARAM_MEASCONFIG1,COUNT1 , &response);
  //=======================================================
  writeParam(SI1133_PARAM_ADCCONFIG2,RATE_NORMAL| F_WHITE, &response );
  writeParam(SI1133_PARAM_ADCSENS2,0x00, &response);
  writeParam(SI1133_PARAM_ADCPSOT2,BITS_16, &response);
  writeParam(SI1133_PARAM_MEASCONFIG2,COUNT1 , &response);
  //=======================================================
  writeParam(SI1133_PARAM_ADCCONFIG3,RATE_NORMAL| F_UV_DEEP, &response );
  writeParam(SI1133_PARAM_ADCSENS3,0x60, &response); //0x60 correspond to 64 data acquisition summed together
  writeParam(SI1133_PARAM_ADCPSOT3,BITS_16, &response);
  writeParam(SI1133_PARAM_MEASCONFIG3,COUNT1 , &response);


  si1133_write8(SI1133_REG_COMMAND, SI1133_START);
  SI1133_delay_msek(100);
 //canal0 =uv
 //canal1 =full ir 
  return true;
}

void si1133_reset() {
//creo q falta reiniciar el irqstatus
  si1133_write8(SI1133_REG_COMMAND, SI1133_RESET_SW);
  SI1133_delay_msek(100);
}
/*********************************************************************/

u16 si1133_readUV() {
	u8 temp=0;
	u16 uv=0;
	si1133_read8(SI1133_REG_HOSTOUT0, &temp);
	uv=temp;
	si1133_read8(SI1133_REG_HOSTOUT1, &temp);
	uv= (uv<<8) + temp;
 	return (u16)(1.5625 * (float)uv); //divide by 64 and multiply by 100
}
u16 si1133_readIR() {
	u8 temp=0;
	u16 ir=0;
	si1133_read8(SI1133_REG_HOSTOUT2, &temp);
	ir=temp;
	si1133_read8(SI1133_REG_HOSTOUT3, &temp);
	ir=(ir<<8) + (temp);
 	return ir;
}

u16 si1133_readVIS() {
	u8 temp=0;
	u16 vis=0;
	si1133_read8(SI1133_REG_HOSTOUT4, &temp);
	vis=temp;
	si1133_read8(SI1133_REG_HOSTOUT5, &temp);
	vis= (vis<<8) + temp;
 	return vis;
}

u16 si1133_readDEEP_UV() {
	u8 temp=0;
 	u16 deep=0;
 	si1133_read8(SI1133_REG_HOSTOUT6, &temp);
 	deep=temp;
 	si1133_read8(SI1133_REG_HOSTOUT7, &temp);
 	deep= (deep<<8) + temp;
 	return (u16)(1.5625 * (float)deep); //divide by 64 and multiply by 100
}
