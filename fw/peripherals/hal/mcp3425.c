///*
// * mcp3425.c
// *
// *  Created on: Dec 2, 2015
// *      Author: Eloi
// */
//
//#include "mcp3425.h"
//#include "../../../Board.h"
//#include "i2c_helper.h"
//#include <xdc/runtime/Timestamp.h>
////#include <xdc/runtime/Types.h>
//#include <msp432.h> //to access the registers
////#include <driverlib/timer_a.h>
//
///* DriverLib Includes */
//#include "driverlib.h"
//
//#define MCP_OUTPUT_LENGTH 3
//#define MCP_WRITE_LENGTH 1
//#define MCP_ADDR Board_MCP3425AD_I2CADDR
//#define MCP_REG_POS 2
//#define MSP_MSB_POS 0
//#define MCP_CONT_MODE 0b00010000
//#define MCP_1SHOT_MODE 0b00000000
//#define MCP_SPS_240 0b00000000
//#define MCP_SPS_60 0b00000100
//#define MCP_SPS_15 0b00001000
//#define MCP_PGA_1 0b00000000
//#define MCP_PGA_2 0b00000001
//#define MCP_PGA_4 0b00000010
//#define MCP_PGA_8 0b00000011
//#define V_MINUS 0
//#define UV_SLOPE 8
//#define UV_OFFSET 8
//#define NOP10() __asm__("nop;nop;nop;nop;nop;nop;nop;nop;nop;nop")
//
//
//int8_t mcp_read (uint8_t dev_addr, uint8_t* reg_data, uint8_t cnt) {
//
//    I2C_Transaction i2cTransaction;
//
//    i2cTransaction.slaveAddress = dev_addr; //MCP3425AD_I2CADDR
//    i2cTransaction.writeBuf = NULL;
//    i2cTransaction.writeCount = 0;
//    i2cTransaction.readBuf = reg_data;
//    i2cTransaction.readCount = cnt;
//
//
//    int8_t ret = I2C_transfer(i2c_helper_handle, &i2cTransaction);
//
//    if (!ret) {
//        //serial_printf(cli_stdout, "mcp3425 read error \n", 0);
//    }
//
//    return ret;
//}
//
//int8_t mcp_write (uint8_t dev_addr, uint8_t* reg_data, uint8_t cnt) {
//    I2C_Transaction i2cTransaction;
//
//    i2cTransaction.slaveAddress = dev_addr; //MCP3425AD_I2CADDR
//    i2cTransaction.readBuf = NULL;
//    i2cTransaction.readCount = 0;
//    i2cTransaction.writeBuf = reg_data;
//    i2cTransaction.writeCount = cnt;
//
//    int8_t ret = I2C_transfer(i2c_helper_handle, &i2cTransaction);
//
//    if (!ret) {
//        //serial_printf(cli_stdout, "SHT2x i2c bus write error\n", 0);
//    }
//
//    return ret;
//}
//
//void mcp_init () {
//    /*
//     * =========================================
//     * bit 4: 1 = continious conversion mode, 0 = one-shot conversion mode
//     * =========================================
//     * bit 3-2: S1-S0 sample rate selection bits:
//     * 00 = 240 SPS (12 bits),
//     * 01 = 60 SPS (14 bits),
//     * 10 = 15 SPS (16 bits)
//     * =========================================
//     * bit 1-0: G1-G0: PGA Gain Selector Bits
//     * 00 = 1 V/V,
//     * 01 = 2 V/V,
//     * 10 = 4 V/V,
//     * 11 = 8 V/V
//     * =========================================
//     * bits 6-5 are not used in this device
//     * bit 7 is the NOTRDY bit and not relevant for initialisation
//     * =========================================
//     */
//
//    /* Here we put it in 1shot conversion mode for energy economy purposes
//     * because the device uses 0.1 micro-A when in sleep mode
//     */
//
//    uint8_t read_buffer[MCP_OUTPUT_LENGTH];
//    uint8_t write_buffer = 0;
//
//    mcp_read(MCP_ADDR, read_buffer, MCP_OUTPUT_LENGTH);
//
//    write_buffer = read_buffer[MCP_REG_POS];
//
//    //set bit 4-0 to 0 and preserve bit 7-5
//    write_buffer = write_buffer & 0b11100000;
//
//    //load the desired config
//    write_buffer = write_buffer + MCP_1SHOT_MODE + MCP_SPS_240 + MCP_PGA_1;
//    mcp_write(MCP_ADDR, &write_buffer, MCP_WRITE_LENGTH);
//}
//
//void mcp_parse (uint8_t* reg_data, cpt_data* parsed_info) {
//
//    int16_t voltage = 0;
//
//    voltage += reg_data[MSP_MSB_POS];
//    voltage = (voltage << 8);
//    voltage |= reg_data[MSP_MSB_POS+1];
//
//
//    parsed_info->uv_data = voltage;
//    parsed_info->status_register = reg_data[MCP_REG_POS];
//}
//
//float mcp_convert_uv_data (int16_t raw_data){
//    //the raw data is, in mV, the difference between Vin+ and Vin- (assuming 240SPS)
//    //If the difference is greater than +/-2.048V, the captor return 2.048V.
//    //Currently we have V_MINUS = 0, so we can't detect the full range of the UV output
//
//    float converted = 0;
//
//    //we adjust the raw data to fit our 0-3.3V range
//    converted = ((float)raw_data)*0.001 + V_MINUS;
//
//    //conversion to mW, the constant are choosen using the datasheet of the UV captor
//    //It may be better to experimentally determine them
//    converted = converted*UV_SLOPE - UV_OFFSET;
//
//    return converted;
//}
//
//float mcp_get_data (){
//
//	uint16_t i = 0;
//    cpt_data uv_cpt;
//    uint8_t buffer[3];
//
//    //turn on UV sensor then wait 2msec
//    GPIO_write(Board_UV_INT, Board_UV_ON);
//    Task_sleep(2);
//
//    mcp_read(MCP_ADDR, buffer, 3);
//
//    //trigger conversion
//    buffer[2] |= 0x80;
//    mcp_write(MCP_ADDR, buffer+2, 1);
//
//    //poll to see if conversion is finished
//    while(1){
//        mcp_read(MCP_ADDR, buffer, 3);
//        if ((buffer[2] & 0x80) == 0x00)
//            break;
//    }
//
//    //Turn UV sensor off
//    GPIO_write(Board_UV_PIN, Board_UV_OFF);
//
//    mcp_parse (buffer, &uv_cpt);
//    uv_cpt.uv_value = mcp_convert_uv_data (uv_cpt.uv_data);
//
//    return uv_cpt.uv_value;
//}
