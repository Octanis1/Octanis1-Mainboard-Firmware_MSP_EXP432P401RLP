/* Example code copyright notice:
 *  --COPYRIGHT--,BSD_EX
 * Copyright (c) 2012, Texas Instruments Incorporated
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
 *
 *******************************************************************************
 *
 *                       MSP430 CODE EXAMPLE DISCLAIMER
 *
 * MSP430 code examples are self-contained low-level programs that typically
 * demonstrate a single peripheral function or device feature in a highly
 * concise manner. For this the code may rely on the device's power-on default
 * register values and settings such as the clock configuration and care must
 * be taken when combining code from several examples to avoid potential side
 * effects. Also see www.ti.com/grace for a GUI- and www.ti.com/msp430ware
 * for an API functional library-approach to peripheral configuration.
 *
 * --/COPYRIGHT--*/
//******************************************************************************
//  MSP430G2x44 EPS code - USCI_B0 I2C Slave and control unit
//
//  Description: This code implements all necessary sensory, control and
//  communication functions of the Octanis 1 rover EPS (Electrical Power Sub-
//  system). It reads out battery and solar cell currents and voltages with
//  the on-chip ADCs, communicates with the MSP432 mainboard master through I2C
//  and receives commands to turn on or off certain modules, i.e. voltage
//  supplies. If necessary, it can even shut down the mainboard and remain in low-
//  power mode until the battery has recharged enough.
//
//                                /|\  /|\
//               MSP430G2x44      10k  10k     MSP432 mainboard
//                   slave         |    |        master
//             -----------------   |    |  -----------------
//           -|XIN  P3.1/UCB0SDA|<-|---+->|SDA 		    XIN|-
//            |                 |  |      |                 |
//           -|XOUT             |  |      |             XOUT|-
//            |     P3.2/UCB0SCL|<-+----->|SCL		       |
//            |                 |         |                 |
//        >---|P2.3/A3          |
//        >---|P2.2/A2          |
//        >---|P2.1/A1          |
//        >---|P2.0/A0          |
//
//  Raffael Tschui
//  Octanis
//  March 2016
//  Built with CCS Version: 6.1.1.00028 for Mac OS X
//******************************************************************************
#include <msp430.h>
#include <stdint.h>
#include "eps_hal.h"
#include "communication.h"

void init_eps()
{
	//init GPIO
	gpio_init();

	//init i2c
	i2c_init();

	//init adc
	ADC_init();

	//init timers
	timer0_A_init();

	//enable mainboard
	//TODO

	//enable interrupts
	__bis_SR_register(GIE);
}

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer

    int mainboard_poke_counter = 0;
	
	init_eps();

	while (1) //execute once, then enter sleep mode until timer interrupt
	{
		mainboard_poke_iterate(&mainboard_poke_counter);
		ADC_update();
		//i2c_update();
		//eps_update();
		//i2c_respond();
		__bis_SR_register(CPUOFF + GIE);        // Enter LPM0 w/ interrupts
		__no_operation();                       // Set breakpoint >>here<< and read
	}
}
