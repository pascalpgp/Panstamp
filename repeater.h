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
 * Creation date: 10/01/2012
 */

#ifndef _REPEATER_H
#define _REPEATER_H

#include "Arduino.h"
#include "swpacket.h"
#include "config.h"

/**
 * Maximum hop
 */
extern byte maxHopCount;

/**
 * Enable flag
 */
extern bool RepeaterEnabled;

/**
 * init
 *
 * Initialize repeater
 *
 * 'maxHop': maximum hop count
 */
void RepeaterInit(byte);

void RepeaterStart();

void RepeaterStop();

/**
 * packetHandler
 *
 * Handle incoming packet. Repeat if necessary
 *
 * 'packet': Pointer to the SWAP packet received
 */
void RepeaterPacketHandler(SWPACKET *);

#endif

