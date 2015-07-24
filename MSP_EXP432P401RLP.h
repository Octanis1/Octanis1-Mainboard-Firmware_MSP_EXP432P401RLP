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
    MSP_EXP432P401RLP_S1 = 0,
    MSP_EXP432P401RLP_S2,
    MSP_EXP432P401RLP_LED1,
    MSP_EXP432P401RLP_LED_RED,

    /*
     * MSP_EXP432P401RLP_LED_GREEN & MSP_EXP432P401RLP_LED_BLUE are used for
     * PWM examples.  Uncomment the following lines if you would like to control
     * the LEDs with the GPIO driver.
     */
    //MSP_EXP432P401RLP_LED_GREEN,
    //MSP_EXP432P401RLP_LED_BLUE,

    MSP_EXP432P401RLP_GPIOCOUNT
} MSP_EXP432P401RLP_GPIOName;

/*!
 *  @def    MSP_EXP432P401RLP_I2CName
 *  @brief  Enum of I2C names on the MSP_EXP432P401RLP dev board
 */
typedef enum MSP_EXP432P401RLP_I2CName {
    MSP_EXP432P401RLP_I2CB0 = 0,

    MSP_EXP432P401RLP_I2CCOUNT
} MSP_EXP432P401RLP_I2CName;

/*!
 *  @def    MSP_EXP432P401RLP_PWMName
 *  @brief  Enum of PWM names on the MSP_EXP432P401RLP dev board
 */
typedef enum MSP_EXP432P401RLP_PWMName {
    MSP_EXP432P401RLP_PWM_TA1_1 = 0,
    MSP_EXP432P401RLP_PWM_TA1_2,

    MSP_EXP432P401RLP_PWMCOUNT
} MSP_EXP432P401RLP_PWMName;

/*!
 *  @def    MSP_EXP432P401RLP_SDSPIName
 *  @brief  Enum of SDSPI names on the MSP_EXP432P401RLP dev board
 */
typedef enum MSP_EXP432P401RLP_SDSPIName {
    MSP_EXP432P401RLP_SDSPIB0 = 0,

    MSP_EXP432P401RLP_SDSPICOUNT
} EMSP_EXP432P401RLP_SDSPIName;

/*!
 *  @def    MSP_EXP432P401RLP_SPIName
 *  @brief  Enum of SPI names on the MSP_EXP432P401RLP dev board
 */
typedef enum MSP_EXP432P401RLP_SPIName {
    MSP_EXP432P401RLP_SPIB0 = 0,
    MSP_EXP432P401RLP_SPIB2,

    MSP_EXP432P401RLP_SPICOUNT
} MSP_EXP432P401RLP_SPIName;

/*!
 *  @def    MSP_EXP432P401RLP_UARTName
 *  @brief  Enum of UART names on the MSP_EXP432P401RLP dev board
 */
typedef enum MSP_EXP432P401RLP_UARTName {
    MSP_EXP432P401RLP_UARTA0 = 0, //"backchannel" port - jumpered
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

/*!
 *  @brief  Initialize board specific WiFi settings
 *
 *  This function initializes the board specific WiFi settings and then calls
 *  the WiFi_init API to initialize the WiFi module.
 *
 *  The hardware resources controlled by the WiFi module are determined by the
 *  WiFi_config variable.
 *
 *  A SimpleLink CC3100 device or module is required and must be connected to
 *  use the WiFi driver.
 */
extern void MSP_EXP432P401RLP_initWiFi(void);

#ifdef __cplusplus
}
#endif

#endif /* __MSP_EXP432P401RLP_H */
