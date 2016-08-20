/*
  AS3935.cpp - AS3935 Franklin Lightning Sensor™ IC by AMS library
  Copyright (c) 2012 Raivis Rengelis (raivis [at] rrkb.lv). All rights reserved.
  Modified by Raffael Tschui (raffael [at] octanis.org) for TI-RTOS I2C Support.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 3 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "../../../Board.h"
#include "i2c_helper.h"
#include "AS3935.h"
#include "driverlib.h"

// i2c address is defined as Board_AS3935_I2CADDR

void AS3935_ccr_ISR(uint16_t timestamp, uint8_t edgetype);
Timer_Handle timer;

const Timer_A_ContinuousModeConfig continuousModeConfig =
{
        TIMER_A_CLOCKSOURCE_ACLK,           		// ACLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_1,      		// ACLK/1 = 32.768khz
        TIMER_A_TAIE_INTERRUPT_DISABLE,     		// Enable Overflow ISR ???
        TIMER_A_DO_CLEAR                    		// Clear Counter
};

Timer_A_CaptureModeConfig captureModeConfig =
{
		Board_ULTRASONIC_IN_0_CCR,        		// CC Register for the first input pin to be initialized.
		TIMER_A_CAPTUREMODE_RISING_AND_FALLING_EDGE,
		TIMER_A_CAPTURE_INPUTSELECT_CCIxA,        // CCIxB Input Select
        TIMER_A_CAPTURE_SYNCHRONOUS,              // Synchronized Capture
        TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE,  // Enable interrupt
        TIMER_A_OUTPUTMODE_OUTBITVALUE            // Output bit value
};

uint16_t pulse_rising_time; //record timestamp when pulse was sent out
uint16_t pulse_falling_time; //record timestamp when the interrupt was triggered by the echoed pulse

void AS3935_init()
{
	/* configure input pins as capture/compare functions: */
	GPIO_setAsPeripheralModuleFunctionInputPin(Board_LIGHTNING_IN_PORT,
			Board_LIGHTNING_IN_PIN,  Board_LIGHTNING_IN_SELECT);

	/* configure the timer as continuous Mode and capture mode for each input pin separately */
	captureModeConfig.captureRegister = Board_LIGHTNING_IN_CCR;
	MAP_Timer_A_configureContinuousMode(Board_LIGHTNING_IN_TAx_MODULE, &continuousModeConfig);
	Timer_A_initCapture(Board_LIGHTNING_IN_TAx_MODULE, &captureModeConfig);

	/* Enabling global timer interrupts and starting timer */
	MAP_Interrupt_enableInterrupt(INT_TA0_N);
	/* Starting the Timer_A0 in continuous mode */
	MAP_Timer_A_startCounter(Board_LIGHTNING_IN_TAx_MODULE, TIMER_A_CONTINUOUS_MODE);
    /* Enable each CCR interrupt */
	Timer_A_enableCaptureCompareInterrupt(Board_LIGHTNING_IN_TAx_MODULE, Board_LIGHTNING_IN_CCR );
	/* Enabling MASTER interrupts */
	MAP_Interrupt_enableMaster();
}

uint8_t _rawRegisterRead(uint8_t reg_addr)
{
	I2C_Transaction i2cTransaction;

	int8_t iError = 0;
	uint8_t readBuffer;

	i2cTransaction.writeBuf = &reg_addr;
	i2cTransaction.writeCount = sizeof(reg_addr);

	i2cTransaction.readBuf = &readBuffer;
	i2cTransaction.readCount = sizeof(readBuffer);

	i2cTransaction.slaveAddress = Board_AS3935_I2CADDR;

	int ret = I2C_transfer(i2c_helper_handle, &i2cTransaction);

	if (!ret) {
		iError = -1;
	}else{
		iError = 0;
	}

	return readBuffer;
}

uint8_t _ffsz(uint8_t mask)
{
	uint8_t i = 0;
	if (mask)
		for (i = 1; ~mask & 1; i++)
			mask >>= 1;
	return i;
}

void registerWrite(uint8_t reg_addr, uint8_t mask, uint8_t data)
{
	uint8_t regval = _rawRegisterRead(reg_addr);
	regval &= ~(mask);
	if (mask)
		regval |= (data << (_ffsz(mask)-1));
	else
		regval |= data;

	I2C_Transaction i2cTransaction;

	int8_t iError = 0;
	uint8_t writebuffer[2];
	writebuffer[0] = reg_addr;
	writebuffer[1] = regval;

	i2cTransaction.readBuf = NULL;
	i2cTransaction.readCount = 0;
	i2cTransaction.writeBuf = writebuffer;
	i2cTransaction.writeCount = 2;
	i2cTransaction.slaveAddress = Board_AS3935_I2CADDR;

	int ret = I2C_transfer(i2c_helper_handle, &i2cTransaction);

	if (!ret) {
		iError = -1;
	}else{
		iError = 0;
	}

}

uint8_t registerRead(uint8_t reg, uint8_t mask)
{
	uint8_t regval = _rawRegisterRead(reg);
	regval = regval & mask;
	if (mask)
		regval >>= (_ffsz(mask)-1);
	return regval;
}

void lightning_reset()
{
	registerWrite(AS3935_PRESET_DEFAULT, AS3935_DIRECT_COMMAND);
	Task_sleep(2); //delay 2ms
}

uint8_t lightning_calibrate()
{
	int target = 3125, currentcount = 0, bestdiff = INT_MAX, currdiff = 0;
	uint8_t bestTune = 0, currTune = 0;
	unsigned long setUpTime;
	int currIrq, prevIrq;
	// set lco_fdiv divider to 0, which translates to 16
	// so we are looking for 31250Hz on irq pin
	// and since we are counting for 100ms that translates to number 3125
	// each capacitor changes second least significant digit
	// using this timing so this is probably the best way to go

	// we count the frequency in a while loop, manually. therefore, disable the interrupt pin
	GPIO_disableInt(Board_LIGHTNING_INT);

	registerWrite(AS3935_LCO_FDIV,0);
	registerWrite(AS3935_DISP_LCO,1);
	// tuning is not linear, can't do any shortcuts here
	// going over all built-in cap values and finding the best
	for (currTune = 0; currTune <= 0x0F; currTune++)
	{
		registerWrite(AS3935_TUN_CAP,currTune);
		// let it settle (and let other tasks run in the meantime.)
		Task_sleep(2); //delay(2);
		currentcount = 0;
		//prevIrq = digitalRead(_IRQPin);

//		setUpTime = Timestamp_get32() + CYCLES_PER_MS*100;

		Task_sleep(100); //		while((long)(Timestamp_get32() - setUpTime) < 0)
//		{
//			currIrq = digitalRead(_IRQPin);
//			if (currIrq > prevIrq)
//			{
//				currentcount++;
//			}
//			prevIrq = currIrq;
//		}


		currdiff = target - currentcount;
		// don't look at me, abs() misbehaves
		if(currdiff < 0)
			currdiff = -currdiff;
		if(bestdiff > currdiff)
		{
			bestdiff = currdiff;
			bestTune = currTune;
		}
	}

	//TODO: this is the empirically found best value.
	//change calibration procedure once with a input capture pin which
	//can actually measure the frequency in a better way
	bestTune = 0x0F;

	registerWrite(AS3935_TUN_CAP,bestTune);
	Task_sleep(2); //delay(2);
	registerWrite(AS3935_DISP_LCO,0);
	// and now do RCO calibration
	powerUp();
	// if error is over 109, we are outside allowed tuning range of +/-3.5%
	return bestdiff > 109?false:true;
}

void tune(int tuneValue)
{
	Task_sleep(100);//delay(100);
	registerWrite(AS3935_TUN_CAP,tuneValue);
	Task_sleep(2);//delay(2);
	powerUp();
}

void powerDown()
{
	registerWrite(AS3935_PWD,1);
}

void powerUp()
{
	registerWrite(AS3935_PWD,0);
	registerWrite(AS3935_CALIB_RCO,AS3935_DIRECT_COMMAND); //_SPITransfer2(0x3D, 0x96);
	Task_sleep(2);//delay(2);
	registerWrite(AS3935_DISP_TRCO,1);
	Task_sleep(2);//delay(2);
	registerWrite(AS3935_DISP_TRCO,0);
	Task_sleep(3);//delay(3);
}

int interruptSource()
{
	return registerRead(AS3935_INT);
}

void disableDisturbers()
{
	registerWrite(AS3935_MASK_DIST,1);
}

void enableDisturbers()
{
	registerWrite(AS3935_MASK_DIST,0);
}

int getMinimumLightnings()
{
	return registerRead(AS3935_MIN_NUM_LIGH);
}

int setMinimumLightnings(int minlightning)
{
	registerWrite(AS3935_MIN_NUM_LIGH,minlightning);
	return getMinimumLightnings();
}

int lightningDistanceKm()
{
	return registerRead(AS3935_DISTANCE);
}

void setIndoors()
{
	registerWrite(AS3935_AFE_GB,AS3935_AFE_INDOOR);
}

void setOutdoors()
{
	registerWrite(AS3935_AFE_GB,AS3935_AFE_OUTDOOR);
}

int getNoiseFloor()
{
	return registerRead(AS3935_NF_LEV);
}

int setNoiseFloor(int noisefloor)
{
	registerWrite(AS3935_NF_LEV,noisefloor);
	return getNoiseFloor();
}

int getSpikeRejection()
{
	return registerRead(AS3935_SREJ);
}

int setSpikeRejection(int srej)
{
	registerWrite(AS3935_SREJ, srej);
	return getSpikeRejection();
}

int getWatchdogThreshold()
{
	return registerRead(AS3935_WDTH);
}

int setWatchdogThreshold(int wdth)
{
	registerWrite(AS3935_WDTH,wdth);
	return getWatchdogThreshold();
}

void clearStats()
{
	registerWrite(AS3935_CL_STAT,1);
	registerWrite(AS3935_CL_STAT,0);
	registerWrite(AS3935_CL_STAT,1);
}

void as3935_ISR()
{
 //TODO

}

/*****************************************************************************************
 * ASM3935_ccr_ISR() gets called in the TIMER_Ax ISR if triggered by AS3935 int pin (p2.0)
 * Arguments:
 * *index : ultrasonic input pin that triggered the CCR (0-7)
 * *timestamp : ...
 * *edgetype : 1=rising, 0=falling
 *****************************************************************************************/
void AS3935_ccr_ISR(uint16_t timestamp, uint8_t edgetype)
{
	if(edgetype)
	{//bit is high -> rising edge
		pulse_rising_time=timestamp;
	}
	else
	{//input bit is low -> falling edge
		pulse_falling_time=timestamp;
	}
}
