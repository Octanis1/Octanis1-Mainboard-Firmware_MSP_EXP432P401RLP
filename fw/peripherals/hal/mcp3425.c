/*
 * mcp3425.c
 *
 *  Created on: Dec 2, 2015
 *      Author: Eloi
 */

#include "mcp3425.h"
#include "../../../Board.h"
#include "i2c_helper.h"
#include <xdc/runtime/Timestamp.h>
//#include <xdc/runtime/Types.h>
#include <msp432.h> //to access the registers
//#include <driverlib/timer_a.h>

/* DriverLib Includes */
#include "driverlib.h"

#define MCP_OUTPUT_LENGTH 3
#define MCP_WRITE_LENGTH 1
#define MCP_ADDR Board_MCP3425AD_I2CADDR
#define MCP_REG_POS 2
#define MSP_MSB_POS 0
#define MCP_CONT_MODE 0b00010000
#define MCP_1SHOT_MODE 0b00000000
#define MCP_SPS_240 0b00000000
#define MCP_SPS_60 0b00000100
#define MCP_SPS_15 0b00001000
#define MCP_PGA_1 0b00000000
#define MCP_PGA_2 0b00000001
#define MCP_PGA_4 0b00000010
#define MCP_PGA_8 0b00000011


char mcp_read (unsigned char dev_addr, unsigned char* reg_data, unsigned char cnt) {
	
    I2C_Transaction i2cTransaction;
	
    i2cTransaction.slaveAddress = dev_addr; //MCP3425AD_I2CADDR
    i2cTransaction.writeBuf = NULL;
    i2cTransaction.writeCount = 0;
    i2cTransaction.readBuf = reg_data;
    i2cTransaction.readCount = cnt;


    int ret = I2C_transfer(i2c_helper_handle, &i2cTransaction);

    if (!ret) {
        //cli_printf("mcp3425 read error \n", 0);
    }

    return ret;
}

char mcp_write (unsigned char dev_addr, unsigned char* reg_data, unsigned char cnt) {
    I2C_Transaction i2cTransaction;

    i2cTransaction.slaveAddress = dev_addr; //MCP3425AD_I2CADDR
    i2cTransaction.readBuf = NULL;
    i2cTransaction.readCount = 0;
    i2cTransaction.writeBuf = reg_data;
    i2cTransaction.writeCount = cnt;

    int ret = I2C_transfer(i2c_helper_handle, &i2cTransaction);

    if (!ret) {
        //cli_printf("SHT2x i2c bus write error\n", 0);
    }

    return ret;
}

void mcp_init () {
    /*
     * =========================================
     * bit 4: 1 = continious conversion mode, 0 = one-shot conversion mode
     * =========================================
     * bit 3-2: S1-S0 sample rate selection bits:
     * 00 = 240 SPS (12 bits),
     * 01 = 60 SPS (14 bits),
     * 10 = 15 SPS (16 bits)
     * =========================================
     * bit 1-0: G1-G0: PGA Gain Selector Bits
     * 00 = 1 V/V,
     * 01 = 2 V/V,
     * 10 = 4 V/V,
     * 11 = 8 V/V
     * =========================================
     * bits 6-5 are not used in this device
     * bit 7 is the NOTRDY bit and not relevant for initialisation
     * =========================================
     */

    unsigned char read_buffer[MCP_OUTPUT_LENGTH];
    unsigned char write_buffer = 0;

    mcp_read(MCP_ADDR, read_buffer, MCP_OUTPUT_LENGTH);

    write_buffer = read_buffer[MCP_REG_POS];
    
    //set bit 4-0 to 0 and preserve bit 7-5
    write_buffer = write_buffer & 0b11100000;
    
    //load the desired config
    write_buffer = write_buffer + MCP_1SHOT_MODE + MCP_SPS_240 + MCP_PGA_2;
    mcp_write(MCP_ADDR, &write_buffer, MCP_WRITE_LENGTH);
}

void mcp_parse (unsigned char* reg_data, cpt_data* parsed_info) {
    
    int voltage = 0;
    
    voltage += reg_data[MSP_MSB_POS];
    voltage = (voltage << 8);
    voltage |= reg_data[MSP_MSB_POS+1];

    //if int is 4 bytes long we have to fill the 2 bigger bytes with the MSB
    if (sizeof(int) == 4){
        if((reg_data[0] & 0x80) == 0x80)
            voltage |= 0xFFFF0000;
        else
            voltage &= 0x0000FFFF;
    }
    
    parsed_info->uv_data = voltage;
    parsed_info->status_register = reg_data[MCP_REG_POS];
}

float mcp_convert_uv_data (int raw_data){
    
    float converted = 0;

    return converted;
}

float mcp_get_data (){

    cpt_data uv_cpt;
    unsigned char buffer[3];

    //turn on UV captor then wait 1msec
    GPIO_write(Board_UV_PIN, Board_UV_ON);
    Task_sleep(1200);
    
    mcp_read(MCP_ADDR, buffer, 3);

    //trigger conversion
    buffer[2] |= 0x80;
    mcp_write(MCP_ADDR, buffer+2, 1);

    //poll to see if conversion is finished
    while(1){
        mcp_read(MCP_ADDR, buffer, 3);
        if ((buffer[2] & 0x80) == 0x00)
            break;
    }

    //Turn UV captor off
    GPIO_write(Board_UV_PIN, Board_UV_OFF);

    mcp_parse (buffer, &uv_cpt);
    uv_cpt.uv_value = mcp_convert_uv_data (uv_cpt.uv_data);

    return uv_cpt.uv_data;
}
