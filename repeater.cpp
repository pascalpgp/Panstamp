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
* Creation date: 10/02/2012
*/

#include "swpacket.h"
#include "panstamp.h"

/**
* Maximum hop
*/
byte maxHopCount;

/**
* Enable flag
*/
bool RepeaterEnabled;

/**
*
*/
void RepeaterStart()
{
	RepeaterEnabled = true;
}

void RepeaterStop()
{
	RepeaterEnabled = false;
}

/**
* init
*
* Initialize repeater
*
* 'maxHop': maximum hop count
*/
void RepeaterInit(byte maxHop)
{
	maxHopCount = maxHop;
	RepeaterStart();
}

/**
* packetHandler
*
* Handle incoming packet. Repeat if necessary
*
* 'packet': Pointer to the SWAP packet received
*/
void RepeaterPacketHandler(SWPACKET * packet)
{
	if (RepeaterEnabled)
	{
		// Don't repeat packets addressed to our device
		if (packet->destAddr != panstamp.cc1101.devAddress)
		{
			// Don't repeat packets send by our device itself (and repeated by another device)
			if (packet->regAddr != panstamp.cc1101.devAddress)
			{
				// Don't repeat beyond the maximum hop count
				if (packet->hop < maxHopCount)
				{
					packet->srcAddr = panstamp.cc1101.devAddress;   // Modify source address
					packet->hop++;                                  // Increment hop counter

					packet->send(
					SWAP_NUMBER_OF_RETRIES,						// Number of retries
					SWAP_DELAY_BEFORE_REPEATING,				// Delay before sending
					SWAP_DELAY_BETWEEN_RETRIES);				// Delay between retries
				}
			}
		}
	}
}
