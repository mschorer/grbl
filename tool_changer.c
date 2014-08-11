/*
  tool_changer.c - tool changer control methods
  Part of Grbl

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


void tools_init()
{
}


void tools_select( uint8_t index)
{
  if (sys.state == STATE_CHECK_MODE) { return; }
}


void tools_change()
{
  if (sys.state == STATE_CHECK_MODE) { return; }
}

