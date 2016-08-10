///*
// * mcp3425.h
// *
// *  Created on: Dec 2, 2015
// *      Author: Eloi
// */
//
//#ifndef FW_PERIPHERALS_HAL_MCP3425_H_
//#define FW_PERIPHERALS_HAL_MCP3425_H_
//
//#include <stdint.h>
//
//typedef struct cpt_data {
//    float uv_value; //mW/cm^2
//    int16_t uv_data;
//    uint8_t status_register;
//}cpt_data;
//
//int8_t mcp_read(uint8_t dev_addr, uint8_t* reg_data, uint8_t cnt);
///*
// * fetch the value of the converted data and the register of the mcp3425
// * return value determine if transaction was successful
// */
//
//int8_t mcp_write (uint8_t dev_addr, uint8_t *reg_data, uint8_t cnt);
///*
// * write to the register of the mcp3425
// * return value determine if transaction was successful
// */
//
//void mcp_init ();
///*
// * initialise the device, putting him in one-shot conversion mode and
// * setting the conversion mode, SPS and gain to the appropriate value
// */
//
//void mcp_parse (uint8_t* reg_data, cpt_data* parsed_info);
///*
// * convert the buffer recived with the i2c for the cpt_data struct.
// */
//
//float mcp_convert_uv_data (int16_t raw_data);
///*
// * Convert the tension recived in mW/cm^2
// * The value returned by the UV captor varie depending on the outside temperature
// * This function is currently uncalibrated for North Pole weather
// * Expect up to 20% error
// */
//
//float mcp_get_data ();
///*
// * turn on the UV captor, wait for him to be ready, fetch the data, turn the UV off, convert the data
// * return value is in mW/cm^2
// */
//
//#endif /* FW_PERIPHERALS_HAL_MCP3425_H_ */
