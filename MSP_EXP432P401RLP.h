/*
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/** ============================================================================
 *  @file       MSP_EXP432P401RLP.h
 *
 *  @brief      MSP_EXP432P401RLP Board Specific APIs
 *
 *  The MSP_EXP432P401RLP header file should be included in an application as
 *  follows:
 *  @code
 *  #include <MSP_EXP432P401RLP.h>
 *  @endcode
 *
 *  ============================================================================
 */
#ifndef __MSP_EXP432P401RLP_H
#define __MSP_EXP432P401RLP_H

//#define VERSION_0_5 // define to use board v0.5 pin definitions, else undefine
//#define VERSION_0_6
#define VERSION_1

#define FLASH_ENABLED 1
#define CONTINUE_WAYPOINTS_IMMEDIATELY 1
#define ARM_IMMEDIATELY	1 //if defined, armed state will not wait for GPS and IMU valid.
#define SBC_ENABLED	// if defined, it will wait for SBC to boot before sending mavlink messages and continue after waypoint 0.
#define LORA_ENABLED 1
#define 	ROCKBLOCK_ENABLED 1
#define EPS_ENABLED 1
//#define BLE_ENABLED 1
#define MAVLINK_ON_UART0_ENABLED 1
//#define MAVLINK_ON_LORA_ENABLED 1 //for verbose lora outputs
//#define UARTCAM_ENABLED 1
//#define GSM_ENABLED 1
//#define USE_ONBOARD_BNO055 1 //in contrast to just decoding incoming mavlink attitude messages.
#define USE_GPS_HEADING 1
//#define USE_ULTRASONIC 1

#ifdef VERSION_0_5
	#warning "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< Use board v0.5 >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"
#else
	#ifndef VERSION_0_6
		#warning "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Use board v1.0 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!>"
	#else
		#warning ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Use board v0.6 >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"
	#endif
#endif


#ifdef __cplusplus
extern "C" {
#endif

/* LEDs on MSP_EXP432P401RLP are active high. */
#define MSP_EXP432P401RLP_LED_OFF (0)
#define MSP_EXP432P401RLP_LED_ON  (1)

/*!
 *  @def    MSP_EXP432P401RLP_GPIOName
 *  @brief  Enum of GPIO names on the MSP_EXP432P401RLP dev board
 */
typedef enum MSP_EXP432P401RLP_GPIOName {
	Octanis_LIGHTNING_INT,
	Octanis_ROCKBLOCK_NET,
	Octanis_GEIGER_COUNTER,
	Octanis_EPS_INT,
	Octanis_UV_INT,
	Octanis_LED0, //octanis led1
	Octanis_LED1, //octanis led2
	Octanis_ROCKBLOCK_SLEEP,
	Octanis_LORA_RESET_N,
#ifdef VERSION_1
	Octanis_5V_EXT_ENABLE, // ultrasonic supply voltage
#endif
	Octanis_ULTRASONIC_TRIGGER1,
	Octanis_ULTRASONIC_TRIGGER0,
	Octanis_WINDSENSOR_SLEEP,
	Octanis_M1234_SLEEP_N,
	Octanis_M1_PH,
	Octanis_M2_PH,
	Octanis_M3_PH,
	Octanis_M4_PH,
	Octanis_M1_EN,
	Octanis_M2_EN,
	Octanis_M3_EN,
	Octanis_M4_EN,
	Octanis_M5678_ON,
	Octanis_M5_PH,
	Octanis_M6_PH,
	Octanis_M7_PH,
	Octanis_M8_PH,

	Octanis_M5678_CURR_SENS_EN,
	Octanis_M1_ANGLE_ENCODER_CS,
	Octanis_GEIGER_EN,
	Octanis_Flash_CS,
#ifndef VERSION_0_5
	Octanis_GPS_POWER_MODE,
#endif
	Octanis_OBS_A_EN,

    MSP_EXP432P401RLP_GPIOCOUNT
} MSP_EXP432P401RLP_GPIOName;

/*!
 *  @def    MSP_EXP432P401RLP_I2CName
 *  @brief  Enum of I2C names on the MSP_EXP432P401RLP dev board
 */
typedef enum MSP_EXP432P401RLP_I2CName {
    Octanis_I2CB0 = 0,

    MSP_EXP432P401RLP_I2CCOUNT
} MSP_EXP432P401RLP_I2CName;

/*!
 *  @def    MSP_EXP432P401RLP_PWMName
 *  @brief  Enum of PWM names on the MSP_EXP432P401RLP dev board
 */
typedef enum MSP_EXP432P401RLP_PWMName {
#ifdef VERSION_0_5
	Octanis_PWM_TA1_2 = 0,
	Octanis_PWM_TA1_1,
	Octanis_PWM_TA1_3,
	Octanis_PWM_TA1_4,
#else
	Octanis_PWM_TA1_1 = 0,
	Octanis_PWM_TA1_2,
	Octanis_PWM_TA1_3,
	Octanis_PWM_TA1_4,
#endif

    MSP_EXP432P401RLP_PWMCOUNT
} MSP_EXP432P401RLP_PWMName;

///*!
// *  @def    MSP_EXP432P401RLP_SDSPIName
// *  @brief  Enum of SDSPI names on the MSP_EXP432P401RLP dev board
// */
//typedef enum MSP_EXP432P401RLP_SDSPIName {
//    Octanis_SD_SPI_UCB0 = 0,
//
//    MSP_EXP432P401RLP_SDSPICOUNT
//} EMSP_EXP432P401RLP_SDSPIName;

/*!
 *  @def    MSP_EXP432P401RLP_SPIName
 *  @brief  Enum of SPI names on the MSP_EXP432P401RLP dev board
 */
typedef enum MSP_EXP432P401RLP_SPIName {
    Octanis_SPIB1 = 0,

    Octanis_SPICOUNT
} Octanis_SPIName;

/*!
 *  @def    MSP_EXP432P401RLP_UARTName
 *  @brief  Enum of UART names on the MSP_EXP432P401RLP dev board
 */
typedef enum MSP_EXP432P401RLP_UARTName {
    MSP_EXP432P401RLP_UARTA0 = 0, //"backchannel" port - jumpered
    MSP_EXP432P401RLP_UARTA1, //TX: P2.3, RX: P2.2
    MSP_EXP432P401RLP_UARTA2, //TX: P3.3, RX: P3.2
	MSP_EXP432P401RLP_UARTA3, //TX: P9.7, RX: P9.6

    MSP_EXP432P401RLP_UARTCOUNT
} MSP_EXP432P401RLP_UARTName;

/*!
 *  @def    MSP_EXP432P401RLP_WatchdogName
 *  @brief  Enum of Watchdog names on the MSP_EXP432P401RLP dev board
 */
typedef enum MSP_EXP432P401RLP_WatchdogName {
    MSP_EXP432P401RLP_WATCHDOG = 0,

    MSP_EXP432P401RLP_WATCHDOGCOUNT
} MSP_EXP432P401RLP_WatchdogName;

/*!
 *  @def    MSP_EXP432P401RLP_WiFiName
 *  @brief  Enum of WiFi names on the MSP_EXP432P401RLP dev board
 */
typedef enum MSP_EXP432P401RLP_WiFiName {
    MSP_EXP432P401RLP_WIFI = 0,

    MSP_EXP432P401RLP_WIFICOUNT
} MSP_EXP432P401RLP_WiFiName;

/*!
 *  @brief  Initialize the general board specific settings
 *
 *  This function initializes the general board specific settings.
 */
extern void MSP_EXP432P401RLP_initGeneral(void);

/*!
 *  @brief  Initialize board specific GPIO settings
 *
 *  This function initializes the board specific GPIO settings and
 *  then calls the GPIO_init API to initialize the GPIO module.
 *
 *  The GPIOs controlled by the GPIO module are determined by the GPIO_PinConfig
 *  variable.
 */
extern void MSP_EXP432P401RLP_initGPIO(void);

/*!
 *  @brief  Initialize board specific I2C settings
 *
 *  This function initializes the board specific I2C settings and then calls
 *  the I2C_init API to initialize the I2C module.
 *
 *  The I2C peripherals controlled by the I2C module are determined by the
 *  I2C_config variable.
 */
extern void MSP_EXP432P401RLP_initI2C(void);

/*!
 *  @brief  Initialize board specific PWM settings
 *
 *  This function initializes the board specific PWM settings and then calls
 *  the PWM_init API to initialize the PWM module.
 *
 *  The PWM peripherals controlled by the PWM module are determined by the
 *  PWM_config variable.
 */
extern void MSP_EXP432P401RLP_initPWM(void);

/*!
 *  @brief  Initialize board specific SDSPI settings
 *
 *  This function initializes the board specific SDSPI settings and then calls
 *  the SDSPI_init API to initialize the SDSPI module.
 *
 *  The SDSPI peripherals controlled by the SDSPI module are determined by the
 *  SDSPI_config variable.
 */
extern void MSP_EXP432P401RLP_initSDSPI(void);

/*!
 *  @brief  Initialize board specific SPI settings
 *
 *  This function initializes the board specific SPI settings and then calls
 *  the SPI_init API to initialize the SPI module.
 *
 *  The SPI peripherals controlled by the SPI module are determined by the
 *  SPI_config variable.
 */
extern void MSP_EXP432P401RLP_initSPI(void);

/*!
 *  @brief  Initialize board specific UART settings
 *
 *  This function initializes the board specific UART settings and then calls
 *  the UART_init API to initialize the UART module.
 *
 *  The UART peripherals controlled by the UART module are determined by the
 *  UART_config variable.
 */
extern void MSP_EXP432P401RLP_initUART(void);

/*!
 *  @brief  Initialize board specific Watchdog settings
 *
 *  This function initializes the board specific Watchdog settings and then
 *  calls the Watchdog_init API to initialize the Watchdog module.
 *
 *  The Watchdog peripherals controlled by the Watchdog module are determined
 *  by the Watchdog_config variable.
 */
extern void MSP_EXP432P401RLP_initWatchdog(void);


#ifdef __cplusplus
}
#endif

#endif /* __MSP_EXP432P401RLP_H */
