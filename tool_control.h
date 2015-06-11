/*
  tool_changer.h - tool control methods
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

#ifndef tool_changer_h
#define tool_changer_h

#define N_TOOL_TABLE 9  // Number of supported tools + 1, #0 is used as a zero-offset tool

typedef struct {
  float r;         // tool radius
  float xyz[3];    // tool offsets for X,Y,Z axes
} gc_tools_t;

// Initialize tool changer
void tools_init();

// select tools
void tool_select(uint8_t index);

// perform change
void tool_change(uint8_t index);

#endif
