/*
  tool_changer.c - tool changer control methods
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
#include "tool_changer.h"
#include "protocol.h"
#include "gcode.h"
#include "i2c_master.h"
#include "settings.h"
#include <stdio.h>

volatile uint8_t selected_tool = 0;

void tools_init()
{
	uint8_t idx = 0;
	  
	TWI_buffer_out[idx++] = CMD_MESSAGE;

	idx = sPrintString( TWI_buffer_out, idx, "GRBL V");
	idx = sPrintString( TWI_buffer_out, idx, GRBL_VERSION);
	TWI_buffer_out[idx++] = ' ';
	idx = sPrintString( TWI_buffer_out, idx, GRBL_VERSION_BUILD);
//	idx = sPrintString( TWI_buffer_out, idx, GRBL_VERSION_BUILD);

	TWI_master_start_write( 0x5c, idx);
}


void tools_select( uint8_t index)
{
  if (sys.state == STATE_CHECK_MODE) return;
  
  selected_tool = index & 0x0f;

  TWI_buffer_out[0] = CMD_TOOL_SELECT | selected_tool;
  TWI_master_start_write( 0x5c, 1);
}


void tools_change( uint8_t index)
{
  if (sys.state == STATE_CHECK_MODE) return;
  
  selected_tool = index & 0x0f;

  uint8_t idx = 0;
  TWI_buffer_out[idx++] = CMD_TOOL_CHANGE | selected_tool;
  
  TWI_buffer_out[idx++] = CMD_MESSAGE;
  TWI_buffer_out[idx++] = 'T';
  TWI_buffer_out[idx++] = 48 + index;
  TWI_buffer_out[idx++] = ' ';
  TWI_buffer_out[idx++] = 'R';
  idx = sPrintFloat( TWI_buffer_out, idx, gc_state.tool_table[index].r, 2);
//  TWI_buffer_out[idx++] = 'X';
//  idx = sPrintFloat( TWI_buffer_out, idx, gc_state.tool_table[index].xyz[0], 2);
//  TWI_buffer_out[idx++] = 'Y';
//  idx = sPrintFloat( TWI_buffer_out, idx, gc_state.tool_table[index].xyz[1], 2);
  TWI_buffer_out[idx++] = ' ';
  TWI_buffer_out[idx++] = 'Z';
  idx = sPrintFloat( TWI_buffer_out, idx, gc_state.tool_table[index].xyz[2], 2);

  TWI_master_start_write( 0x5c, idx);
}

uint8_t sPrintFloat( unsigned char *out, uint8_t idx, float n, uint8_t decimal_places)
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
	while(a > 0) {
		if (i == decimal_places) buf[i++] = '.';
		buf[i++] = (a % 10) + '0'; // Get digit
		a /= 10;
	}
	while (i < decimal_places) {
		buf[i++] = '0'; // Fill in zeros to decimal point for (n < 1)
	}
	if (i == decimal_places) { // Fill in leading zero, if needed.
		buf[i++] = '0';
	}
	
	for (; i > 0; i--, idx++)
	  out[ idx] = buf[i-1];
	
	return idx;
}

uint8_t sPrintString( unsigned char *out, uint8_t idx, unsigned char *str)
{
	uint8_t i=0;
	
	while ( str[ i] != 0 && i < 30) {
		out[idx++] = str[ i++]; // Fill in zeros to decimal point for (n < 1)
	}
	
	return idx;
}