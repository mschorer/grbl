/*
  tool_changer.c - tool changer control methods
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
#include "tool_control.h"
#include "machine_control.h"
#include "protocol.h"
#include "gcode.h"
#include "settings.h"
#include <stdio.h>

volatile uint8_t selected_tool = 0;

void tools_init()
{
	tool_change( 0);
}


void tool_select( uint8_t index)
{
  if (sys.state == STATE_CHECK_MODE) return;
  
  selected_tool = index & 0x0f;

  mctrl_queueCmd( CMD_TX | selected_tool);
}


void tool_change( uint8_t index)
{
  if (sys.state == STATE_CHECK_MODE) return;
  
  tool_select( index);

  mctrl_queueCmd( CMD_M6);
  mctrl_queueMsgTool( index);
}
