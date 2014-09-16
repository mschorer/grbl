/*
  machine_control.h - speaking to the machine
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

#ifndef machine_control_h
#define machine_control_h

#define MCTRL_I2C_ADDR 0x5c

#define MCTRL_CMDBUF_LEN 16
#define MCTRL_MSGBUF_LEN 42

// Initialize tool changer
void mctrl_init();

// select tools
void mctrl_select(uint8_t index);

// try to send buffer
bool mctrl_flush();

void mctrl_queueCmd( uint8_t b);
void mctrl_queueInt( uint16_t b);

void mctrl_queueChar( char b);
void mctrl_queueFloat( float n, uint8_t decimal_places);
void mctrl_queueString( const char *str);

#endif
