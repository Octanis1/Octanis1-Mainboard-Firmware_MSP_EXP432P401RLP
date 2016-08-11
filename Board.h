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

#ifndef __BOARD_H
#define __BOARD_H

#define VERBOSE	1

#ifdef __cplusplus
extern "C" {
#endif

#include "MSP_EXP432P401RLP.h"

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/utils/Load.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/hal/Seconds.h>
#include <ti/sysbios/hal/Timer.h>
#include <ti/sysbios/knl/Mailbox.h>
#include <ti/sysbios/knl/Queue.h>


/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/PWM.h>
#include <ti/drivers/SPI.h>

#include <stdint.h>
#include <string.h>
#include <ctype.h>
#include <math.h>


/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>

#include "fw/core/cli.h" //enable CLI printf everywhere

#define MAVLINK_SYSTEM_ID	25

/* Clock cycle conversion factors */
#define CYCLES_PER_US		48			// cycles per microsecond
#define CYCLES_PER_MS		48000		// cycles per millisecond
#define CYCLES_PER_S		48000000	// cycles per second


/* Board specific I2C addresses */
#define Board_EPS_I2CADDR					(0x48)
#define Board_BMP180_I2CADDR					(0x77) //temp/pres
#define Board_BMP280_I2CADDR					(0x77) //temp/pres
#define Board_BNO055_MAINBOARD_I2CADDR			(0x28) //IMU 1
#define Board_BNO055_WEATHERSTRIP_I2CADDR		(0x29) //IMU 2
#define Board_MCP3425AD_I2CADDR					(0x68) //ADC for UV
#define Board_SHT21_I2CADDR						(0x40) //temp/hygro
#define Board_BME280_I2CADDR					(0x76) //hygro/pres
#define Board_AS3935_I2CADDR					(0x03) //lightning

/* The default I�C address of the BNO055 device is (0x29).
 * The alternative address (0x28), can be selected by pulling COM3 down.
 */

/* EPS pins */
#define Board_EPS_ALIVE_REQ_IV		DIO_PORT_IV__IFG1 // pin 2.1

/* Board specific SPI CS pins */
#define Board_Flash_CS				Octanis_Flash_CS



/* General board mappings */
#define Board_initGeneral           MSP_EXP432P401RLP_initGeneral
#define Board_initGPIO              MSP_EXP432P401RLP_initGPIO
#define Board_initI2C               MSP_EXP432P401RLP_initI2C
#define Board_initPWM               MSP_EXP432P401RLP_initPWM
#define Board_initSDSPI             MSP_EXP432P401RLP_initSDSPI
#define Board_initSPI               MSP_EXP432P401RLP_initSPI
#define Board_initUART              MSP_EXP432P401RLP_initUART
#define Board_initWatchdog          MSP_EXP432P401RLP_initWatchdog

#define Board_LED_ON                MSP_EXP432P401RLP_LED_ON
#define Board_LED_OFF               MSP_EXP432P401RLP_LED_OFF

//#define Board_BUTTON0               MSP_EXP432P401RLP_S1
//#define Board_BUTTON1               MSP_EXP432P401RLP_S2
#define Board_LED_RED               	Octanis_LED0
#define Board_LED_GREEN              Octanis_LED1

#define Board_ROCKBLOCK_SLEEP		Octanis_ROCKBLOCK_SLEEP
#define Board_ROCKBLOCK_NET			Octanis_ROCKBLOCK_NET

/* LoRa module pin definitions */
#define Board_LORA_RESET_N			Octanis_LORA_RESET_N


/*Ultrasonic pin and timer definitions*/
#define Board_ULTRASONIC_ENABLE			Octanis_5V_EXT_ENABLE
#define Board_ULTRASONIC_TRIGGER0		Octanis_ULTRASONIC_TRIGGER0
#define Board_ULTRASONIC_TRIGGER1		Octanis_ULTRASONIC_TRIGGER1

#define Board_ULTRASONIC_IN_0_PORT		GPIO_PORT_P6
#define Board_ULTRASONIC_IN_0_PIN		GPIO_PIN6
#define Board_ULTRASONIC_IN_0_SELECT		GPIO_PRIMARY_MODULE_FUNCTION
#define Board_ULTRASONIC_IN_0_TAx_MODULE	TIMER_A2_BASE
#define Board_ULTRASONIC_IN_0_CCR		TIMER_A_CAPTURECOMPARE_REGISTER_3
#define Board_ULTRASONIC_IN_0_CCTL		TIMER_A2->CCTL[3]
#define Board_ULTRASONIC_IN_0_IV			0x06

#define Board_ULTRASONIC_IN_1_PORT		GPIO_PORT_P5
#define Board_ULTRASONIC_IN_1_PIN		GPIO_PIN7
#define Board_ULTRASONIC_IN_1_SELECT		GPIO_PRIMARY_MODULE_FUNCTION
#define Board_ULTRASONIC_IN_1_TAx_MODULE	TIMER_A2_BASE
#define Board_ULTRASONIC_IN_1_CCR		TIMER_A_CAPTURECOMPARE_REGISTER_2
#define Board_ULTRASONIC_IN_1_CCTL		TIMER_A2->CCTL[2]
#define Board_ULTRASONIC_IN_1_IV			0x04

#define Board_ULTRASONIC_IN_2_PORT		GPIO_PORT_P9
#define Board_ULTRASONIC_IN_2_PIN		GPIO_PIN2
#define Board_ULTRASONIC_IN_2_SELECT		GPIO_PRIMARY_MODULE_FUNCTION
#define Board_ULTRASONIC_IN_2_TAx_MODULE	TIMER_A3_BASE
#define Board_ULTRASONIC_IN_2_CCR		TIMER_A_CAPTURECOMPARE_REGISTER_3
#define Board_ULTRASONIC_IN_2_CCTL		TIMER_A3->CCTL[3]
#define Board_ULTRASONIC_IN_2_IV			0x06

#define Board_ULTRASONIC_IN_3_PORT		GPIO_PORT_P9
#define Board_ULTRASONIC_IN_3_PIN		GPIO_PIN3
#define Board_ULTRASONIC_IN_3_SELECT		GPIO_PRIMARY_MODULE_FUNCTION
#define Board_ULTRASONIC_IN_3_TAx_MODULE	TIMER_A3_BASE
#define Board_ULTRASONIC_IN_3_CCR		TIMER_A_CAPTURECOMPARE_REGISTER_4
#define Board_ULTRASONIC_IN_3_CCTL		TIMER_A3->CCTL[4]
#define Board_ULTRASONIC_IN_3_IV			0x08

#define Board_ULTRASONIC_IN_4_PORT		GPIO_PORT_P5
#define Board_ULTRASONIC_IN_4_PIN		GPIO_PIN6
#define Board_ULTRASONIC_IN_4_SELECT		GPIO_PRIMARY_MODULE_FUNCTION
#define Board_ULTRASONIC_IN_4_TAx_MODULE	TIMER_A2_BASE
#define Board_ULTRASONIC_IN_4_CCR		TIMER_A_CAPTURECOMPARE_REGISTER_1
#define Board_ULTRASONIC_IN_4_CCTL		TIMER_A2->CCTL[1]
#define Board_ULTRASONIC_IN_4_IV			0x02

#define Board_ULTRASONIC_IN_5_PORT		GPIO_PORT_P6
#define Board_ULTRASONIC_IN_5_PIN		GPIO_PIN7
#define Board_ULTRASONIC_IN_5_SELECT		GPIO_PRIMARY_MODULE_FUNCTION
#define Board_ULTRASONIC_IN_5_TAx_MODULE	TIMER_A2_BASE
#define Board_ULTRASONIC_IN_5_CCR		TIMER_A_CAPTURECOMPARE_REGISTER_4
#define Board_ULTRASONIC_IN_5_CCTL		TIMER_A2->CCTL[4]
#define Board_ULTRASONIC_IN_5_IV			0x08

//remapped port!
#define Board_ULTRASONIC_IN_6_PORT		GPIO_PORT_P7
#define Board_ULTRASONIC_IN_6_PIN		GPIO_PIN0
#define Board_ULTRASONIC_IN_6_SELECT		GPIO_PRIMARY_MODULE_FUNCTION
#define Board_ULTRASONIC_IN_6_TAx_MODULE	TIMER_A0_BASE
#define Board_ULTRASONIC_IN_6_CCR		TIMER_A_CAPTURECOMPARE_REGISTER_2
#define Board_ULTRASONIC_IN_6_CCTL		TIMER_A0->CCTL[2]
#define Board_ULTRASONIC_IN_6_IV			0x04

//remapped port!
#define Board_ULTRASONIC_IN_7_PORT		GPIO_PORT_P7
#define Board_ULTRASONIC_IN_7_PIN		GPIO_PIN1
#define Board_ULTRASONIC_IN_7_SELECT		GPIO_PRIMARY_MODULE_FUNCTION
#define Board_ULTRASONIC_IN_7_TAx_MODULE	TIMER_A0_BASE
#define Board_ULTRASONIC_IN_7_CCR		TIMER_A_CAPTURECOMPARE_REGISTER_3
#define Board_ULTRASONIC_IN_7_CCTL		TIMER_A0->CCTL[3]
#define Board_ULTRASONIC_IN_7_IV			0x06

/* UV sensor pins */
#define Board_UV_INT			Octanis_UV_INT
#define 	Board_UV_INT_PORT	GPIO_PORT_P1
#define 	Board_UV_INT_PIN		GPIO_PIN5
#define Board_UV_ON 			1
#define Board_UV_OFF			0

/* Windsensor pins */
#define Board_WINDSENSOR_SLEEP			Octanis_WINDSENSOR_SLEEP

#define Board_WINDSENSOR_IN_PORT			GPIO_PORT_P7 //P7.3 (TIMER_A0->CCR[1] port mapped in PWM_init())
#define Board_WINDSENSOR_IN_PIN			GPIO_PIN3
#define Board_WINDSENSOR_IN_SELECT		GPIO_PRIMARY_MODULE_FUNCTION
#define Board_WINDSENSOR_IN_TAx_MODULE	TIMER_A0_BASE
#define Board_WINDSENSOR_IN_CCR			TIMER_A_CAPTURECOMPARE_REGISTER_1
#define Board_WINDSENSOR_IN_CCTL			TIMER_A0->CCTL[1]
#define Board_WINDSENSOR_IN_IV			0x02


/* Lightning sensor */
#define Board_LIGHTNING_INT			Octanis_LIGHTNING_INT
#ifdef VERSION_1
	#define Board_LIGHTNING_INT_IV		DIO_PORT_IV__IFG0
#else
	#define Board_LIGHTNING_INT_IV		DIO_PORT_IV__IFG4
#endif

/*Geiger counter */
#define Board_GEIGER_COUNTER		Octanis_GEIGER_COUNTER
#define Board_GEIGER_EN				Octanis_GEIGER_EN
#define Board_GEIGER_ON 1
#define Board_GEIGER_OFF 0

/*Debug pin to see when he switches to obstacle avoidance mode*/
#define Board_OBS_A_EN				Octanis_OBS_A_EN

/* Motor pins */
#define Board_M1234_SLEEP_N			Octanis_M1234_SLEEP_N
#define Board_M1_PH					Octanis_M1_PH
#define Board_M2_PH  				Octanis_M2_PH
#define Board_M3_PH 					Octanis_M3_PH
#define Board_M4_PH					Octanis_M4_PH
#define Board_M1_EN					Octanis_M1_EN
#define Board_M2_EN					Octanis_M2_EN
#define Board_M3_EN					Octanis_M3_EN
#define Board_M4_EN 					Octanis_M4_EN

#define Board_M1_ANGLE_ENCODER_CS	Octanis_M1_ANGLE_ENCODER_CS

#define Board_M5678_ON	 			Octanis_M5678_ON
#define Board_M5_PH					Octanis_M5_PH
#define Board_M6_PH					Octanis_M6_PH
#define Board_M7_PH					Octanis_M7_PH
#define Board_M8_PH					Octanis_M8_PH

#ifdef VERSION_0_5
	#define Board_M5_EN_PWM           	Octanis_PWM_TA1_2
	#define Board_M6_EN_PWM           	Octanis_PWM_TA1_1
	#define Board_M7_EN_PWM           	Octanis_PWM_TA1_3
	#define Board_M8_EN_PWM           	Octanis_PWM_TA1_4
#else
	#define Board_M5_EN_PWM           	Octanis_PWM_TA1_1
	#define Board_M6_EN_PWM           	Octanis_PWM_TA1_2
	#define Board_M7_EN_PWM           	Octanis_PWM_TA1_3
	#define Board_M8_EN_PWM           	Octanis_PWM_TA1_4
#endif

#define Board_M5678_CURR_SENS_EN 	Octanis_M5678_CURR_SENS_EN


#define Board_I2C0                  	Octanis_I2CB0
#define Board_SPI                  	Octanis_SPIB1

#define Board_UART0_DEBUG           	MSP_EXP432P401RLP_UARTA0 //"backchannel UART"
#define Board_UART1_GPS             	MSP_EXP432P401RLP_UARTA1 //P9.6,9.7
#define Board_UART2_COMM  			MSP_EXP432P401RLP_UARTA2 //P3.2,3.3   (GSM or ROCKBLOCK)
#define Board_UART3_LORACOMM			MSP_EXP432P401RLP_UARTA3 //P2.2=RX,2.3=TX


#define Board_WATCHDOG0             MSP_EXP432P401RLP_WATCHDOG


#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H */
