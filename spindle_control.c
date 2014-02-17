/*
  spindle_control.c - spindle control methods
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
#include "spindle_control.h"
#include "protocol.h"
#include "i2c_master.h"

void spindle_init()
{    
  spindle_stop();
}

void spindle_stop()
{
    TWI_buffer_out[0] = 0;
    TWI_master_start_write( 0x5c, 1);
}

void spindle_run(uint8_t direction, float rpm) 
{
  // Empty planner buffer to ensure spindle is set when programmed.
  protocol_buffer_synchronize(); 

  // Halt or set spindle direction and rpm. 
  if (direction == SPINDLE_DISABLE) {
  
    spindle_stop();
  
  } else {
/*
    switch (direction) {
    case SPINDLE_ENABLE_CW:
    case SPINDLE_ENABLE_CCW:
    default:
    }
*/
    uint8_t rpm_100 = floor( rpm / 100);

    TWI_buffer_out[0] = rpm_100;
    TWI_master_start_write( 0x5c, 1);
  }
}
