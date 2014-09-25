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

#define CMD_MX	0x00

#define CMD_M3	0x03
#define CMD_M4	0x04
#define CMD_M5	0x05

#define CMD_M6	0x06

#define CMD_M7	0x07
#define CMD_M8	0x08
#define CMD_M9	0x09

#define CMD_TX	0x10

#define CMD_MESSAGE	0x70

#define CMD_SPINDLE_OFF 0x80
#define CMD_SPINDLE_HI 0x8000

// Initialize tool changer
void mctrl_init();

// select tools
void mctrl_select(uint8_t index);

// try to send buffer
bool mctrl_flush();

void mctrl_tick();

void mctrl_queueCmd( uint8_t b);
void mctrl_queueCmdInt( uint16_t b);

void mctrl_queueMsgChar( char b);
void mctrl_queueMsgFloat( float n, uint8_t decimal_places);
void mctrl_queueMsgString( const char *str);

void mctrl_queueMsgTool( uint8_t toolidx);

#endif
