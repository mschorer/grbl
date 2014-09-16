/*
  machine_control.c - speaking to the machine
  Part of Grbl v0.9

  Copyright (c) 2012-2014 Sungeun K. Jeon
  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "system.h"
#include "machine_control.h"
#include "protocol.h"
#include "gcode.h"
#include "i2c_master.h"
#include "settings.h"
#include <stdio.h>


uint8_t mctrl_cmd_len = 0;
uint8_t mctrl_cmd_buf[ MCTRL_CMDBUF_LEN];

uint8_t mctrl_msg_len = 0;
uint8_t mctrl_msg_buf[ MCTRL_MSGBUF_LEN];

void mctrl_init()
{
	mctrl_cmd_len = 0;
	mctrl_msg_len = 0;
	
	mctrl_queueChar( CMD_MESSAGE);

	mctrl_queueString( "GRBL v");
	mctrl_queueString( GRBL_VERSION);
	mctrl_queueString( " ");
	mctrl_queueString( GRBL_VERSION_BUILD);
	//	idx = mctrl_queueString( GRBL_VERSION_BUILD);
	
	mctrl_flush();
}

bool mctrl_flush()
{
	if ( TWI_busy) return false;
	
	uint8_t idx;
	for( idx=0; idx < mctrl_cmd_len; idx++) {
		TWI_buffer_out[ idx] = mctrl_cmd_buf[ idx];
	}
	
	for( uint8_t i = 0; i < mctrl_msg_len; i++, idx++) {
		TWI_buffer_out[ idx] = mctrl_msg_buf[ i];
	}
	
	if ( idx > 0) TWI_master_start_write( MCTRL_I2C_ADDR, idx);

	mctrl_cmd_len = 0;
	mctrl_msg_len = 0;
	
	return true;
}

void mctrl_queueCmd( uint8_t b)
{
	mctrl_cmd_buf[ mctrl_cmd_len++] = b;
}

void mctrl_queueInt( uint16_t b)
{
	mctrl_queueCmd( (b >> 8) & 0xff);
	mctrl_queueCmd( (b & 0xff));
}

void mctrl_queueChar( char b)
{
	mctrl_msg_buf[ mctrl_msg_len++] = b;
}

void mctrl_queueFloat( float n, uint8_t decimal_places)
{
	uint8_t i = 0;
	unsigned char buf[10];

	if (n < 0) {
		buf[ i++] = '-';
		n = -n;
	}

	uint8_t decimals = decimal_places;
	while (decimals >= 2) { // Quickly convert values expected to be E0 to E-4.
		n *= 100;
		decimals -= 2;
	}
	if (decimals) { n *= 10; }
	n += 0.5; // Add rounding factor. Ensures carryover through entire value.
	
	// Generate digits backwards and store in string.
	uint32_t a = (long)n;
	buf[decimal_places] = '.';
	while(a > 0) {
		if (i == decimal_places) i++;
		buf[i++] = (a % 10) + '0'; // Get digit
		a /= 10;
	}
	while (i < decimal_places) {
		buf[i++] = '0'; // Fill in zeros to decimal point for (n < 1)
	}
	if (i == decimal_places) { // Fill in leading zero, if needed.
		i++;
		buf[i++] = '0';
	}
	
	for (; i > 0; i--)
	  mctrl_queueChar( buf[i-1]);
}

void mctrl_queueString( const char *str)
{
	uint8_t i=0;
	
	while ( str[ i] != 0 && i < 30) {
		mctrl_queueChar( str[ i++]); // Fill in zeros to decimal point for (n < 1)
	}
}