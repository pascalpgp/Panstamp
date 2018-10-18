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

#include "panstamp.h"
#include "commonregs.h"
#include "calibration.h"

#if defined(RADINO_CC1101)
	#include "PinChangeInterrupt.h"

	#define attachIRQ_GDO0()	attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(GDO0), isrGDO0event, FALLING);
	#define enableIRQ_GDO0()	enablePinChangeInterrupt(digitalPinToPinChangeInterrupt(GDO0));
	#define disableIRQ_GDO0()	disablePinChangeInterrupt(digitalPinToPinChangeInterrupt(GDO0));
#endif
#if defined(PANSTAMP)

	#define attachIRQ_GDO0()		attachInterrupt(0, isrGDO0event, FALLING);
	#define enableIRQ_GDO0()		attachInterrupt(0, isrGDO0event, FALLING);
	#define disableIRQ_GDO0()       detachInterrupt(0);
#endif

extern unsigned long currentTimeWhenInterrupt;

DEFINE_COMMON_REGINDEX_START()
DEFINE_COMMON_REGINDEX_END()

/**
* Array of registers
*/
extern REGISTER* regTable[];
extern byte		 regTableSize;

/************************************************************************/
/* Definitions relating to the elimination of Duplicates                */
/************************************************************************/
#define LPR_SIZE  32
#define DUPLICATE_EXPIRATION_TIME  5 //000      // Period of time after which packets cannot be considered as duplicates

//LPR stands for Last Packet Received
struct
{
	byte regAddr;             // Register address
	byte function;            // SWAP function
	byte nonce;               // Cyclic nonce
	unsigned long timeStamp;  // Packet Transmission timestamp (ms)
}
LPR[LPR_SIZE];

static byte  LPR_Index;
static byte  LPR_UpdateIndex = 0;

extern bool TxIntervalUpdated;

/**
* PANSTAMP
*
* Class constructor
*/
PANSTAMP::PANSTAMP(void)
{
	statusReceived = NULL;
}

/**
* enableRepeater
*
* Enable repeater mode
*
* 'maxHop'  MAximum repeater count. Zero if omitted
*/
void PANSTAMP::RepeaterEnable(byte maxHop)
{
	RepeaterInit(maxHop);
	cc1101.disableAddressCheck();

	if (maxHop == 0)
		RepeaterStop();
}

/**
* getRegister
*
* Return pointer to register with ID = regId
*
* 'regId'  Register ID
*/
REGISTER * getRegister(byte regId)
{
	if (regId >= regTableSize)
		return NULL;

	return regTable[regId];
}

/**
* isrGDO0event
*
* Event on GDO0 pin (INT0)
*/
void isrGDO0event(void)
{
	bool PacketIsNotDuplicate;

	// Disable interrupt
	disableIRQ_GDO0();
	
	if (panstamp.cc1101.rfState == RFSTATE_RX)
	{
		// To Do : Move these definition out of the context
		static CCPACKET ccPacket;
		static SWPACKET swPacket;
		REGISTER *reg;
		static bool eval;

		eval = true;

		if (panstamp.cc1101.receiveData(&ccPacket) > 6)
		{
			if (ccPacket.crc_ok)
			{
				swPacket = SWPACKET(&ccPacket);

				// Check if packet is duplicate
				PacketIsNotDuplicate = true;

				for (LPR_Index = 0; LPR_Index < LPR_SIZE; LPR_Index++)
				{
					// Same Register Address?
					if (LPR[LPR_Index].regAddr == swPacket.regAddr)
					{
						// Same SWAP function?
						if (LPR[LPR_Index].function == swPacket.function)
						{
							// Same cyclic nonce?
							if (LPR[LPR_Index].nonce == swPacket.nonce)
							{
								if ((panstamp.currentTime - LPR[LPR_Index].timeStamp) < DUPLICATE_EXPIRATION_TIME)
								{
									PacketIsNotDuplicate = false;   //Don't send packet to Gateway
									break;
								}
							}
						}
					}
				}

				// Should Packet be Accepted ?
				if (PacketIsNotDuplicate == true)
				{
					// Repeater enabled?
					if (RepeaterEnabled == true)
					{
						RepeaterPacketHandler(&swPacket);
					}

					// Update LPR with last packet sent (Circular Buffer)
					LPR[LPR_UpdateIndex].regAddr   = swPacket.regAddr;			// Register address
					LPR[LPR_UpdateIndex].function  = swPacket.function;			// SWAP function
					LPR[LPR_UpdateIndex].nonce     = swPacket.nonce;			// Cyclic nonce
					LPR[LPR_UpdateIndex].timeStamp = panstamp.currentTime;		// Current time stamp
					LPR_UpdateIndex				   = (LPR_UpdateIndex + 1) % LPR_SIZE;

					// Smart encryption locally enabled?
					if (panstamp.security & 0x02)
					{
						// OK, then incoming packets must be encrypted too
						if (!(swPacket.security & 0x02))
						{
							eval = false;
						}
					}

					if (eval)
					{
						// Function
						switch(swPacket.function)
						{
							case SWAPFUNCT_CMD:
							// Command not addressed to us?
							if (swPacket.destAddr != panstamp.cc1101.devAddress)
							{
								break;
							}
							// Current version does not support data recording mode
							// so destination address and register address must be the same
							if (swPacket.destAddr != swPacket.regAddr)
							{
								break;
							}
							// Valid register?
							if ((reg = getRegister(swPacket.regId)) == NULL)
							{
								break;
							}
							// Anti-playback security enabled?
							if (panstamp.security & 0x01)
							{
								// Check received nonce
								if (panstamp.nonce != swPacket.nonce)
								{
									// Nonce mismatch. Transmit correct nonce.
									reg = getRegister(REGI_SECUNONCE);
									reg->sendSwapStatus();
									break;
								}
							}

							// Filter incorrect data lengths
							if (swPacket.value.length == reg->length)
							{
								reg->setData(swPacket.value.data);
							}
							else
							{
								reg->sendSwapStatus();
							}
							break;

							case SWAPFUNCT_QRY:
							// Only Product Code can be broad casted
							if (swPacket.destAddr == SWAP_BCAST_ADDR)
							{
								if (swPacket.regId != REGI_PRODUCTCODE)
								{
									break;
								}
							}
							// Query not addressed to us?
							else if (swPacket.destAddr != panstamp.cc1101.devAddress)
							{
								break;
							}

							// Current version does not support data recording mode
							// so destination address and register address must be the same
							if (swPacket.destAddr != swPacket.regAddr)
								break;

							// Valid register?
							if ((reg = getRegister(swPacket.regId)) == NULL)
								break;
							reg->getData();
							break;

							case SWAPFUNCT_STA:
							// User callback function declared ?
							if (panstamp.statusReceived != NULL)
								panstamp.statusReceived(&swPacket);
							break;

							case SWAPFUNCT_ACK:
							{
								// Command not addressed to us?
								if (swPacket.destAddr != panstamp.cc1101.devAddress)
								{
									break;
								}
							// Status message consists of an Acknowledgment for a Packet received by the modem
							// Packet also contains Timestamp and Tx Interval updates
							// To Do : Process Timestamp and Tx Interval
								unsigned long	_Timestamp;

								_Timestamp  = ((unsigned long) swPacket.value.data[0]);
								_Timestamp += ((unsigned long) swPacket.value.data[1]) <<  8;
								_Timestamp += ((unsigned long) swPacket.value.data[2]) << 16;
								_Timestamp += ((unsigned long) swPacket.value.data[3]) << 24;

								// To Do : Update clock speed !
								panstamp.stop_RTC();
								panstamp.start_RTC(_Timestamp);

//								Serial.print("TxInterval = ");
//								unsigned int _TxInterval = (unsigned int)((swPacket.value.data[4] << 8) + swPacket.value.data[5]);
//								Serial.println(_TxInterval, HEX);

//								panstamp.setTxInterval(swPacket.value.data+4, true);

								panstamp.AckReceivedFromModem = true;
								
							}
							break;
							
							default:
							break;
						}
					}
				}
			}
		}
	}
	// Enable GDO0 interrupt
	enableIRQ_GDO0();
}

/**
* start_RTC
*
* Start RTC configured according to the periodicity of Task 0 (default task, similar to previous behavior)
*/
void PANSTAMP::start_RTC(unsigned long timestamp)
{
	// Set timer 2 to asynchronous mode (32.768KHz crystal)
	ASSR = (1 << AS2);

	// Calculate the optimal sleep period in order to minimize power consumption
	// Could start with 8 but easier with 5 for periodicity and lag
	for (timeIncrement=5; timeIncrement>1; timeIncrement--)
	{
		if ((task[0].periodicity % timeIncrement) == 0)
		break;
	}
	// Put Timer2 in CTC mode (using "Output Compare A Match" interrupts)
	TCCR2A = (1 << WGM21);
	// Set Output Compare Register A to value corresponding to the requested frequency
	// Save OCR2A value so that it can be used in the goToSleep loop
	OCR2A_SavedValue = (32 * (timeIncrement - 1)) + 31;
	OCR2A  = OCR2A_SavedValue;
	// Enable Timer2 "Output Compare A Match" interrupts
	TIMSK2 = (1 << OCIE2A);
	// Adjust Timer Counter and currentTime in order to synchronize the RTC
	TCNT2 = (byte)(32 * (timestamp % ((unsigned long) timeIncrement)));
	currentTime =  (timestamp / ((unsigned long) timeIncrement)) * ((unsigned long) timeIncrement);
	// Set Timer2 Prescaler to 1024 in order to start the timer with 1/32 of a second beat
	TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);
	// Wait for all registers to be updated
	while (ASSR & B00011111) {}
}

/**
* stop_RTC
*
* Stop RTC by setting TCCR2B to 0, stopping Timer2
*/
void PANSTAMP::stop_RTC()
{
	// Clear TIMSK2 register to disable interrupts
	TIMSK2 = 0;
	// Clear TCCR2B register to stop timer/counter
	TCCR2B = 0;
	// Wait for TCCR2 register to be updated
	while (ASSR & (1 << TCR2BUB)) {}
}

/**
* set number of tasks
*
*/
void PANSTAMP::setNumberOfTasks(byte taskNumber)
{
	if (taskNumber < 2)
		taskNumber = 2;
	if (taskNumber >= MAX_NUMBER_OF_TASKS)
		taskNumber = MAX_NUMBER_OF_TASKS;
	numberOfTasks = taskNumber;
}

/**
* set task schedule
*
* This function could be enhanced by adding an offset to be able to schedule a task
* for example, every day at 1:00pm
*/
void PANSTAMP::setTaskSchedule(byte index)
{
	if (index > numberOfTasks) return;
	// This line calculates the next schedule
	noInterrupts();						// Make sure currentTime is not updated while in use
	task[index].nextSchedule = ((currentTime / task[index].periodicity) * task[index].periodicity) + task[index].periodicity + (unsigned long) (task[index].timeLag);
	interrupts();
}

/**
* init
*
* Initialize panStamp board
*/
void PANSTAMP::init()
{
	// Setup CC1101
	cc1101.init();

	for (LPR_Index = 0; LPR_Index < LPR_SIZE ; LPR_Index++)
	{
		LPR[LPR_Index].regAddr   = 0;		// Register address
		LPR[LPR_Index].function  = 0;		// SWAP function
		LPR[LPR_Index].nonce     = 0;		// Cyclic nonce
		LPR[LPR_Index].timeStamp = 0UL;		// Current time stamp
	}

	// Security disabled by default
	security = 0;

	// Set number of tasks to a minimal of 2 (default)
	setNumberOfTasks(2);

	task[0].periodicity = 900UL;
	task[0].timeLag = (5 * (unsigned int) (cc1101.devAddress - 1));
	setTaskSchedule(0);

	// Set Clock Update Schedule (Task 1)
	task[1].periodicity = 86400UL;		// Once a day
	task[1].timeLag = 0;
	setTaskSchedule(1);

	// Enter RX state
	cc1101.setRxState();

	// Attach callback function for GDO0 (INT0)
	attachIRQ_GDO0();

	// Default values
	nonce = 0;
	systemState = SYSTATE_RXON;

	start_RTC(0UL);
}

/**
* reset
*
* Reset panStamp
*/
void PANSTAMP::reset()
{
	stop_RTC();
	// Tell the network that our panStamp is restarting
	systemState = SYSTATE_RESTART;
	getRegister(REGI_SYSSTATE)->sendSwapStatus();

	// Reset panStamp
	wdt_disable();
	wdt_enable(WDTO_15MS);
	while (1) {}
}

/**
* goToSleep
*/
void PANSTAMP::goToSleep(void)
{
	systemState = SYSTATE_RXOFF;

	// Select appropriate sleep mode (Power Save Mode)
	set_sleep_mode(SLEEP_MODE_PWR_SAVE);

	// Power-down CC1101
	cc1101.setPowerDownState();

	// Disable ADC
	ADCSRA &= ~(1 << ADEN);

	// Switch off resources in use (except TIMER2)
	PRR = (1 << PRTWI) | (1 << PRTIM0) | (1 << PRTIM1) | (1 << PRSPI) | (1 << PRUSART0) | (1 << PRADC);

	// Let's assume there isn't any task to launch
	taskToLaunch = 0;
	
	// Let's assume that no resources are required by tasks to be launched
	//	taskResourcesUsed = 0;
	
	// Loop waiting for tasks to launch (or interrupt coming from other devices)
	while (!taskToLaunch)
	{
		interruptByTimer2 = false;

		sleep_enable();
		sleep_mode();										// Enter sleep mode
		sleep_disable();

		if (interruptByTimer2 == true)
		{
			for (taskIndex=0; taskIndex<numberOfTasks; taskIndex++)
			{
				if (currentTime >= task[taskIndex].nextSchedule)
				{
					// Set Task bit in bitmap so that "loop" knows what Task to launch
					taskToLaunch |= (1 << taskIndex);
					// Schedule next time Task should be launched
					task[taskIndex].nextSchedule += task[taskIndex].periodicity;
				}
			}
			// The following 2 lines are very important because they make sure that the Timer2 cycle is ended before
			// looping back to sleep mode, otherwise the interrupt is called several times.
			OCR2A  = OCR2A_SavedValue;
			// Wait for all registers to be updated
			while (ASSR & (1 << OCR2AUB)) {}
		}
	}
	// Re-Power functions
	power_all_enable();									// To be refined with only the mandatory functions

	// Reset bits in Power Reduction Register in order to power up resources in use
	//	PRR = (1 << PRTWI) | (1 << PRTIM0) | (1 << PRTIM1) | (1 << PRSPI) | (1 << PRUSART0) | (1 << PRADC);
	PRR = 0x00;

	// Switch on ADC
	ADCSRA |= (1 << ADEN);

	// Reset CC1101 IC
	cc1101.wakeUp();

	// set system state to RF Rx ON
	systemState = SYSTATE_RXON;
}

/**
* goToIdle
*
* Put PanStamp into Idle State, waiting for a task(s) to be launched
*/
void PANSTAMP::goToIdle(void)
{
	unsigned long timeShot;
	byte taskIndex;

	taskToLaunch = 0; // Assume that there will be no Task to launch
	while (!taskToLaunch)
	{
		// The following lines is required in order to avoid effects of a "currentTime" update during the "while loop"
		noInterrupts();
		timeShot = currentTime;
		interrupts();
		
		for (taskIndex=0; taskIndex<numberOfTasks; taskIndex++)
		{
			if (timeShot >= task[taskIndex].nextSchedule)
			{
				// Set Task bit in bitmap so that "loop" knows what Task to launch
				taskToLaunch |= (1 << taskIndex);
				// Schedule next time Task should be launched
				task[taskIndex].nextSchedule += task[taskIndex].periodicity;
			}
		}
	}
}

/**
* enterSystemState
*
* Enter system state
*
* 'state'  New system state
*/
void PANSTAMP::enterSystemState(SYSTATE state)
{
	// Enter SYNC mode (full Rx mode)
	byte newState[] = {state};
	regTable[REGI_SYSSTATE]->setData(newState);
}

/**
* getInternalTemp
*
* Read internal (ATMEGA328 only) temperature sensor
* Reference: http://playground.arduino.cc/Main/InternalTemperatureSensor
*
* Return:
* 	Temperature in degrees Celsius
*/
long PANSTAMP::getInternalTemp(void)
{
	unsigned int wADC;
	long t;

	// The internal temperature has to be used
	// with the internal reference of 1.1V.
	// Channel 8 can not be selected with
	// the analogRead function yet.

	// Set the internal reference and mux.
	ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX3));
	ADCSRA |= _BV(ADEN);  // enable the ADC

	delay(20);            // wait for voltages to become stable.

	ADCSRA |= _BV(ADSC);  // Start the ADC

	// Detect end-of-conversion
	while (bit_is_set(ADCSRA,ADSC));

	// Reading register "ADCW" takes care of how to read ADCL and ADCH.
	wADC = ADCW;

	// The offset of 324.31 could be wrong. It is just an indication.
	t = (wADC - 324.31 ) / 1.22;

	// The returned temperature is in degrees Celcius.
	return (t);
}

/**
* setTxInterval
*
* Set interval for periodic transmissions
*
* 'interval'	New periodic interval. 0 for asynchronous devices
* 'save'     If TRUE, save parameter in EEPROM
*/
void PANSTAMP::setTxInterval(byte* interval, bool save)
{
//	if (strncmp((char *)interval, (char *)txInterval, 2) == 0)
//		return;

	memcpy(txInterval, interval, sizeof(txInterval));

	// Save in EEPROM
	if (save)
	{
		EEPROM.write(EEPROM_TX_INTERVAL, interval[0]);
		EEPROM.write(EEPROM_TX_INTERVAL + 1, interval[1]);
	}

	task[0].periodicity = (unsigned long)((txInterval[0] << 8) + txInterval[1]);
	task[0].timeLag		= (5 * (unsigned int)(panstamp.cc1101.devAddress - 1));
	setTaskSchedule(0);

	TxIntervalUpdated = true;
}

/**
* setSmartPassword
*
* Set Smart Encryption password
*
* 'password'	Encryption password
*/
void PANSTAMP::setSmartPassword(byte* password)
{
	// Save password
	memcpy(encryptPwd, password, sizeof(encryptPwd));
	// Enable Smart Encryption
	security |= 0x02;
}

/**
* Pre-instantiate PANSTAMP object
*/
PANSTAMP panstamp;

