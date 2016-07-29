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
//return ERROR or SUCCESS
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
  if (id != 0x55) return false; // mira si es si1133
  //device reset at init
  si1133_reset();
  writeParam(SI1133_PARAM_MEASRATEH,0,&response);
  writeParam(SI1133_PARAM_MEASRATEL,1,&response);
  writeParam(SI1133_PARAM_MEASCOUNT0,5,&response);
  writeParam(SI1133_PARAM_MEASCOUNT1,10,&response);
  //seleccionamos los canales 0(16bits) y 1(24bits)
  //por lo que los resultados estaran en
  //HOTSOUT[0-1]------> canal 0 , 2 registros por la resolucion de 16bits
  //HOTSOUT[2-4]------> canal 1
  writeParam(SI1133_PARAM_CHLIST,(uint8_t)0X03,&response);
  //=======================================================
  //configuraciones para el canal 0
  //seleccionamos el rate y el photodiodo
  writeParam(SI1133_PARAM_ADCCONFIG0,RATE_NORMAL| F_UV, &response );

  writeParam(SI1133_PARAM_ADCSENS0,0, &response);
  //resolucion de los datos
  writeParam(SI1133_PARAM_ADCPSOT0,BITS_16, &response);
  writeParam(SI1133_PARAM_MEASCONFIG0,COUNT0, &response);
  //=======================================================
  writeParam(SI1133_PARAM_ADCCONFIG1,RATE_NORMAL| F_LARGE_IR, &response );
  writeParam(SI1133_PARAM_ADCSENS1,0, &response);
  writeParam(SI1133_PARAM_ADCPSOT1,BITS_24, &response);
  writeParam(SI1133_PARAM_MEASCONFIG1,COUNT1 , &response);

  si1133_write8(SI1133_REG_COMMAND, SI1133_START);
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

u32 si1133_readUV() {
	u8 temp;
	u32 uv;
	temp=si1133_read8(SI1133_REG_HOSTOUT0, &temp);
	uv=temp<<8;
	temp=si1133_read8(SI1133_REG_HOSTOUT1, &temp);
	uv= uv | temp;
 	return uv;
}
u32 si1133_readIR() {
	u8 temp;
	u32 ir;
	temp=si1133_read8(SI1133_REG_HOSTOUT2, &temp);
	ir=temp<<16;
	temp=si1133_read8(SI1133_REG_HOSTOUT3, &temp);
	ir=ir | (temp<<8);
	temp=si1133_read8(SI1133_REG_HOSTOUT4, &temp);
	ir=ir|temp;
 	return ir;
}
