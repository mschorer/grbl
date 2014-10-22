/*
  status.h - does a status feedback by LED blink code
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

#ifndef status_h
#define status_h

// Define system header files and standard libraries used by Grbl
#include <inttypes.h>    
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

// Define Grbl configuration and shared header files
#include "config.h"
#include "cpu_map.h"
#include "gcode.h"

// Initialize the serial protocol
void status_init();

// Executes an internal status command, defined as a string starting with a '$'
void status_set( uint8_t status);

#endif
