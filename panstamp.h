/**
 * Copyright (c) 2011 panStamp <contact@panstamp.com>
 * 
 * This file is part of the panStamp project.
 * 
 * panStamp  is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * any later version.
 * 
 * panStamp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with panStamp; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 
 * USA
 * 
 * Author: Daniel Berenguer
 * Creation date: 03/03/2011
 */

#ifndef _PANSTAMP_H
#define _PANSTAMP_H

#include "Arduino.h"
#include "EEPROM.h"
#include "cc1101.h"
#include "nvolat.h"
#include "register.h"
#include "swpacket.h"
#include "config.h"
#include "repeater.h"
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>

/**
 * RTC definitions
 */
#define RTC_250MS    0x03   // Timer 2 prescaler = 32
#define RTC_500MS    0x04   // Timer 2 prescaler = 64
#define RTC_1S       0x05   // Timer 2 prescaler = 128
#define RTC_2S       0x06   // Timer 2 prescaler = 256
#define RTC_8S       0x07   // Timer 2 prescaler = 1024

/*
 * Periodicity of Tasks (Number of tasks is currently limited to 16 because of the taskToLaunch variable)
 */
#define MAX_NUMBER_OF_TASKS	6		// Number of individual Task(s) that can be launch separately or simultaneously when returning from goToSleep

/**
 * Macros
 */
#define setSwapStatusCallBack(ptrFunc)   statusReceived = ptrFunc

#define eepromToFactoryDefaults()                             \
  EEPROM.put(EEPROM_SYNC_WORD,		CC1101_DEFVAL_SYNC1);        \
  EEPROM.put(EEPROM_SYNC_WORD + 1,	CC1101_DEFVAL_SYNC0);    \
  EEPROM.put(EEPROM_DEVICE_ADDR,	CC1101_DEFVAL_ADDR);       \
  EEPROM.put(EEPROM_FREQ_CHANNEL,	CC1101_DEFVAL_CHANNR);    \
  EEPROM.put(EEPROM_TX_INTERVAL,	0xFF);                     \
  EEPROM.put(EEPROM_TX_INTERVAL+1,	0xFF)

// Où les constantes ci-dessus sont-elles définies ?
/*
#define setHighTxPower()		cc1101.setTxPowerAmp(PA_12dBm)		// Maximum for CC1101 chip / 34.2mA
#define setTxPower_10dBm()		cc1101.setTxPowerAmp(PA_10dBm)		// Maximum allowed in Europe / 30mA
#define setTxPower_7dBm()		cc1101.setTxPowerAmp(PA_7dBm)		// 25.8mA
#define setTxPower_5dBm()		cc1101.setTxPowerAmp(PA_5dBm)		// 19.9mA
#define setLowTxPower()			cc1101.setTxPowerAmp(PA_LowPower)	// Default for PanStamp 2-3 dBm / 17mA
#define setTxPower_0dBm()		cc1101.setTxPowerAmp(PA_0dBm)		// 16.8mA
*/

#define enableAntiPlayback()    security |= 0x01

/**
 * System states
 */
enum SYSTATE
{
  SYSTATE_RESTART = 0,
  SYSTATE_RXON,
  SYSTATE_RXOFF,
  SYSTATE_SYNC,
  SYSTATE_LOWBAT
};

/**
 * Class: PANSTAMP
 * 
 * Description:
 * panStamp main class
 */
class PANSTAMP
{
  public:
    /**
     * start_RTC
     *
     * Start RTC based on periodicity of Task 0 (default task, similar to previous behavior)
     *
     */
    void start_RTC(unsigned long timestamp);

	/**
     * stop_RTC
     *
	 * Stop RTC by setting TCCR2B to 0, stopping Timer2
     *
     */
    void stop_RTC();

    /**
     * True if the external 32.768 KHz crystal is enabled
     */
    bool rtcCrystal;
	
	// Acknowledge
	bool AckReceivedFromModem;

    /**
     * CC1101 radio interface
     */
    CC1101 cc1101;
    
    /**
     * Security options
     */
    byte security;

    /**
     * Security cyclic nonce
     */
    byte nonce;
    
    /**
     * System state
     */
    byte systemState;

    /**
     * Interval between periodic transmissions. 0 for asynchronous transmissions
     */
    byte txInterval[2];

    /**
     * Smart encryption password
     */
    byte encryptPwd[12];

    /**
     * CurrentTime (RTC time counter)
     */
	volatile unsigned long currentTime;
	byte timeIncrement;
	byte OCR2A_SavedValue;
	volatile bool interruptByTimer2;
	
    /**
     * Variables relating to the launch of tasks
     */
	byte taskIndex;
	byte numberOfTasks;
	unsigned int taskToLaunch;										// Contains 1 bit per task to launch
	struct {
		unsigned long	nextSchedule;								// Next scheduled time for launching the task
		unsigned long	periodicity;								// Interval between to successive launches of the task
		unsigned int	timeLag;									// Time lag after which the task is launched (de-synchronize)
		unsigned int	usedResources;								// Bitmap of resources used by the task
		} task[MAX_NUMBER_OF_TASKS];

    /**
     * enableRepeater
     *
     * Enable repeater mode
     */
//    void RepeaterEnable(void);

    /**
     * enableRepeater
     *
     * Enable repeater mode
     *
     * 'maxHop'  MAximum repeater count. Zero if omitted
     */
    void RepeaterEnable(byte maxHop=0);

    /**
     * SWAP status packet received. Callback function
     */
    void (*statusReceived)(SWPACKET *status);

    /**
     * PANSTAMP
     *
     * Class constructor
     */
    PANSTAMP(void);

    /**
     * init
     * 
     * Initialize panStamp board
     */
    void init(void);

    /**
     * reset
     * 
     * Reset panStamp
     */
    void reset(void);

	/**
     * set number of tasks
     * 
     */
    void setNumberOfTasks(byte);

	/**
     * set Task Schedule
     * 
     */
    void setTaskSchedule(byte);

    /**
     * wakeUp
     *
     * Wake from sleep mode
     *
     * 'rxOn' Enter RX_ON state after waking up
     */
    void wakeUp(bool rxOn=true);

    /**
     * goToSleep
     *
     * Sleep whilst in power-down mode. This function currently uses sleepWd in a loop
     *
     */
    void goToSleep(void);

    /**
     * goToIdle
     *
     * Put PanStamp into Idle State waiting for task to be launched
     *
     */
    void goToIdle(void);

/**
     * enterSystemState
     *
     * Enter system state
     *
     * 'state'  New system state
     */
    void enterSystemState(SYSTATE state);

    /**
     * getInternalTemp
     * 
     * Read internal (ATMEGA328 only) temperature sensor
     * Reference: http://playground.arduino.cc/Main/InternalTemperatureSensor
     * 
     * Return:
     * 	Temperature in degrees Celsius
     */
    long getInternalTemp(void);

    /**
     * setTxInterval
     * 
     * Set txInterval[] according to parameter and save in EEPROM depending on bool value
     */
	void setTxInterval(byte* interval, bool save);

    /**
     * setSmartPassword
     * 
     * Set Smart Encryption password
     * 
     * 'password'	Encryption password
     */
    void setSmartPassword(byte* password);

    /**
     * enableRfRx
     * 
     * Enable or disable RF reception
     *
     * @param ena Enable if true. Disable otherwise
     */
    void enableRfRx(bool ena = true);
};

/**
 * Global PANSTAMP object
 */
extern PANSTAMP panstamp;

/**
 * getRegister
 *
 * Return pointer to register with ID = regId
 *
 * 'regId'  Register ID
 */
REGISTER * getRegister(byte regId);

#endif
