/*
  machine_control.c - speaking to the machine
  Part of Grbl v0.9

  Copyright (c) 2014 ms@ms-ite.de

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
#include "i2c_tiva.h"

#include "settings.h"
#include <stdio.h>

void mctrl_init()
{
	TWI_init();

	char msg[32];
	sprintf( msg, "%cGRBL v%s %s", CMD_MESSAGE, GRBL_VERSION, GRBL_VERSION_BUILD);
	TWI_putOp( MCTRL_I2C_ADDR, I2C_WRITE, (uint8_t*) msg, strlen( msg), 0);
	TWI_triggerSend();
}

void mctrl_u8Cmd( uint8_t b) {
	TWI_putOp( MCTRL_I2C_ADDR, I2C_WRITE, &b, 1, 0);
	TWI_triggerSend();
}

void mctrl_u16Cmd( uint16_t b) {
	uint8_t val[2];

	val[0] = (b >> 8) & 0xff;
	val[1] = b & 0xff;
	TWI_putOp( MCTRL_I2C_ADDR, I2C_WRITE, val, 2, 0);
	TWI_triggerSend();
}

void mctrl_limitStatus( uint8_t limits) {
	char msg[2];
	// "XYZUVW";
	uint8_t i, mask = 0x01, bit = 0x01;

	msg[0] = CMD_MESSAGE | CMD_MSG_LIM;
	msg[1] = 0;

	for( i = 0; i < 6; i++) {
		if (limits & mask) msg[1] |= bit;
		if ( i == 1) mask <<= 3;
		else mask <<= 1;
	}

	TWI_putOp( MCTRL_I2C_ADDR, I2C_WRITE, (uint8_t*) msg, 2, 0);
}

void mctrl_msgCmd( uint8_t tidx) {
	char msg[32];
	
	sprintf( msg, "%cT%i R%2.2f\nX%3.2f Y%3.2f Z%3.2f", CMD_MESSAGE, tidx, gc_state.tool_table[tidx].r, gc_state.tool_table[tidx].xyz[0], gc_state.tool_table[tidx].xyz[1], gc_state.tool_table[tidx].xyz[2]);
	TWI_putOp( MCTRL_I2C_ADDR, I2C_WRITE, (uint8_t*) msg, strlen( msg), 0);
	TWI_triggerSend();
}
