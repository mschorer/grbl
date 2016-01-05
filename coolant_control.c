/*
  coolant_control.c - coolant control methods
  Part of Grbl v0.9

  Copyright (c) 2012-2014 Sungeun K. Jeon
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
#include "coolant_control.h"
#include "machine_control.h"
#include "protocol.h"
#include "gcode.h"


void coolant_init()
{
#if ( COOLANT_CTRL == CTRL_PIN)
	COOLANT_FLOOD_DDR |= (1 << COOLANT_FLOOD_BIT);
	#ifdef ENABLE_M7
	COOLANT_MIST_DDR |= (1 << COOLANT_MIST_BIT);
	#endif
#endif

	coolant_stop();
}


void coolant_stop()
{
#if ( COOLANT_CTRL == CTRL_PIN)
	COOLANT_FLOOD_PORT &= ~(1 << COOLANT_FLOOD_BIT);
	#ifdef ENABLE_M7
	COOLANT_MIST_PORT &= ~(1 << COOLANT_MIST_BIT);
	#endif
#elif ( COOLANT_CTRL == CTRL_I2C)
	mctrl_u8Cmd( CMD_M9);
#endif
}


void coolant_run(uint8_t mode)
{
  if (sys.state == STATE_CHECK_MODE) { return; }

#if ( COOLANT_CTRL == CTRL_PIN)
	if (mode == COOLANT_FLOOD_ENABLE) {
	COOLANT_FLOOD_PORT |= (1 << COOLANT_FLOOD_BIT);

	#ifdef ENABLE_M7
	} else if (mode == COOLANT_MIST_ENABLE) {
	  COOLANT_MIST_PORT |= (1 << COOLANT_MIST_BIT);
	#endif

	} else {
	coolant_stop();
	}
#elif ( COOLANT_CTRL == CTRL_I2C)
	switch( mode) {
		case COOLANT_MIST_ENABLE: 	mctrl_u8Cmd( CMD_M7); break;
		case COOLANT_FLOOD_ENABLE: 	mctrl_u8Cmd( CMD_M8); break;
		
		default:
			coolant_stop();		
	}
#endif
}
