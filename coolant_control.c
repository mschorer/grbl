/*
  coolant_control.c - coolant control methods
  Part of Grbl

  Copyright (c) 2012-2014 Sungeun K. Jeon

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
#include "i2c_master.h"
#include "protocol.h"

void coolant_init()
{
  coolant_stop();
}


void coolant_stop()
{
    TWI_buffer_out[0] = COOLANT_DISABLE;
//    TWI_master_start_write( 0x5e, 1);
}


void coolant_run(uint8_t mode)
{
  // COOLANT_FLOOD_ENABLE = 1
  // COOLANT_MIST_ENABLE = 2
  TWI_buffer_out[0] = mode;
//  TWI_master_start_write( 0x5e, 1);
}
