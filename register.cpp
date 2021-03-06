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
* Creation date: 04/24/2011
*/

#include "swpacket.h"
#include "panstamp.h"
#include "register.h"
#include "swstatus.h"

byte regIndex = 0;

/**
* getData
*
* Update and get register value
*/
void REGISTER::getData(void)
{
	// Update register value
	if (updateValue != NULL)
	{
		updateValue(id);
	}

	// Send SWAP status message about the new value
	sendSwapStatus();
}

/**
* setData
*
* Set register value
*
* 'data'	New register value
*/
void REGISTER::setData(byte *data)
{
	// Update register value
	if (setValue != NULL)
	{
		setValue(id, data);
	}

	// Send SWAP status message
	sendSwapStatus();
}

/**
* sendSwapStatus
*
* Send SWAP status message
*/
void REGISTER::sendSwapStatus(void)
{
	SWSTATUS packet = SWSTATUS(id, value, length);
	packet.send(SWAP_NUMBER_OF_RETRIES, SWAP_DELAY_BEFORE_SENDING, SWAP_DELAY_BETWEEN_RETRIES);
}

