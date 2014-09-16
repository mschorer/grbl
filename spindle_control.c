/*
  spindle_control.c - spindle control methods
  Part of Grbl v0.9

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
/* 
  This file is based on work from Grbl v0.8, distributed under the 
  terms of the MIT-license. See COPYING for more details.  
    Copyright (c) 2009-2011 Simen Svale Skogsrud
    Copyright (c) 2012 Sungeun K. Jeon
*/ 

#include "system.h"
#include "spindle_control.h"
#include "machine_control.h"
#include "protocol.h"
#include "gcode.h"
#include "i2c_master.h"


void spindle_init()
{    
#if ( SPINDLE_CTRL == CTRL_PIN)
	// On the Uno, spindle enable and PWM are shared. Other CPUs have seperate enable pin.
	#ifdef VARIABLE_SPINDLE
	SPINDLE_PWM_DDR |= (1<<SPINDLE_PWM_BIT); // Configure as PWM output pin.
	#ifndef CPU_MAP_ATMEGA328P
	  SPINDLE_ENABLE_DDR |= (1<<SPINDLE_ENABLE_BIT); // Configure as output pin.
	#endif
	#else
	SPINDLE_ENABLE_DDR |= (1<<SPINDLE_ENABLE_BIT); // Configure as output pin.
	#endif
	SPINDLE_DIRECTION_DDR |= (1<<SPINDLE_DIRECTION_BIT); // Configure as output pin.  spindle_stop();
#endif
    spindle_stop();
}


void spindle_stop()
{
#if ( SPINDLE_CTRL == CTRL_PIN)
	// On the Uno, spindle enable and PWM are shared. Other CPUs have seperate enable pin.
	#ifdef VARIABLE_SPINDLE
	TCCRA_REGISTER &= ~(1<<COMB_BIT); // Disable PWM. Output voltage is zero.
	#ifndef CPU_MAP_ATMEGA328P
	  SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT); // Set pin to low.
	#endif
	#else
	SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT); // Set pin to low.
	#endif
#elif ( SPINDLE_CTRL == CTRL_I2C)
	mctrl_queueCmd( CMD_SPINDLE_OFF);
#endif
}


void spindle_run(uint8_t direction, float rpm) 
{
  if (sys.state == STATE_CHECK_MODE) { return; }
  
  // Halt or set spindle direction and rpm. 
  if (direction == SPINDLE_DISABLE) {
    spindle_stop();
  } else {

#if ( SPINDLE_CTRL == CTRL_PIN)
	if (direction == SPINDLE_ENABLE_CW) {
	  SPINDLE_DIRECTION_PORT &= ~(1<<SPINDLE_DIRECTION_BIT);
	} else {
	  SPINDLE_DIRECTION_PORT |= (1<<SPINDLE_DIRECTION_BIT);
	}

	#ifdef VARIABLE_SPINDLE
	  #define SPINDLE_RPM_RANGE (SPINDLE_MAX_RPM-SPINDLE_MIN_RPM)
	  TCCRA_REGISTER = (1<<COMB_BIT) | (1<<WAVE1_REGISTER) | (1<<WAVE0_REGISTER);
	  TCCRB_REGISTER = (TCCRB_REGISTER & 0b11111000) | 0x02; // set to 1/8 Prescaler
	  rpm -= SPINDLE_MIN_RPM;
	  if ( rpm > SPINDLE_RPM_RANGE ) { rpm = SPINDLE_RPM_RANGE; } // Prevent uint8 overflow
	  uint8_t current_pwm = floor( rpm*(255.0/SPINDLE_RPM_RANGE) + 0.5);
	  OCR_REGISTER = current_pwm;

	  #ifndef CPU_MAP_ATMEGA328P // On the Uno, spindle enable and PWM are shared.
		SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);
	  #endif
	#else
	  SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);
	#endif
#elif ( SPINDLE_CTRL == CTRL_I2C)
	uint16_t rpm_val = min( 32767, rpm);
//	if ( rpm > SPINDLE_RPM_MAX) rpm_val = SPINDLE_RPM_STEPS;
//	else rpm_val = floor( rpm / SPINDLE_RPM_SCALE);

	mctrl_queueInt( CMD_SPINDLE_HI | rpm_val);
#endif
  }
}

