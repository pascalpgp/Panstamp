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
 * Creation date: 07/03/2011
 */

#ifndef _NVOLAT_H
#define _NVOLAT_H

#include "Arduino.h"
#include <EEPROM.h>

/**
 * EEPROM addresses
 */
#define EEPROM_FREQ_CHANNEL			0x0000 // 1-byte register
#define EEPROM_NOT_USED				0x0001 // 1-byte register
#define EEPROM_SYNC_WORD			0x0002 // 2-byte register
#define EEPROM_DEVICE_ADDR			0x0004 // 1-byte register
#define EEPROM_TX_INTERVAL			0x0005 // 2-byte register

#define EEPROM_FIRST_CUSTOM			0x0020
#define EEPROM_DEVICE_UNIQUE_ID		0x0020

#define EEPROM_MANUFACTURING_YEAR	0x0020
#define EEPROM_MANUFACTURING_WEEK	0x0021
#define EEPROM_SERIAL_NUMBER_MSB	0x22
#define EEPROM_SERIAL_NUMBER_LSB	0x23

// DAAT = Device Address Allocation Table
#define EEPROM_AAT_START_ADDRESS	0x0100
#define EEPROM_AAT_SIZE				0x0100 // 256 bytes table

#endif
