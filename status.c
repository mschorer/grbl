/*
  status.c - does a status feedback with LED blink codes
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
#include "nuts_bolts.h"
#include "status.h"
#include "settings.h"
#include "gcode.h"
#include "machine_control.h"

//volatile uint8_t status_state;
//volatile uint8_t status_led;
volatile uint8_t status_ticks;

void status_init()
{
	// set to output
	STATUS_LED_DDR |= (1<<STATUS_LED_BIT);
	//STATUS_LED_PORT |= 1<<STATUS_LED_BIT;

	TCCR2B = 0x00;			//Disable Timer2 while we set it up

	TCNT2	= 1;
	TIFR2   = 0x00;			//Timer2 INT Flag Reg: Clear Timer Overflow Flag
	TIMSK2  = 1 << TOIE2;	//Timer2 INT Reg: Timer2 Overflow Interrupt Enable

	TCCR2A  = 0x00;			//Timer2 Control Reg A: Normal port operation, Wave Gen Mode normal
	TCCR2B  = (1<<CS22) | (1<<CS21) | (1<<CS20);	//0x07;			//Timer2 Control Reg B: Timer Prescaler set to 1024

//	status_state = PROGRAM_FLOW_COMPLETED;
	status_ticks = 62;
}

//Timer2 Overflow Interrupt Vector, called every 1ms
ISR(TIMER2_OVF_vect) {

	mctrl_tick();
	
	status_ticks--;
	if ( status_ticks == 0) {

		if ( ! ( STATUS_LED_IN & (1<<STATUS_LED_BIT))) {
			switch (sys.state) {
				case STATE_IDLE:	status_ticks = 1; break;

				case STATE_QUEUED:	status_ticks = 22; break;
				case STATE_HOLD:	status_ticks = 42; break;

				case STATE_CYCLE:	status_ticks = 62; break;

				case STATE_HOMING:	status_ticks = 8; break;
				case STATE_ALARM:	status_ticks = 8; break;
				case STATE_CHECK_MODE:	status_ticks = 124; break;

				default: status_ticks = 248;
			}

			STATUS_LED_PORT |= 1<<STATUS_LED_BIT;
		} else {
			switch (sys.state) {
				case STATE_IDLE:	status_ticks = 61; break;

				case STATE_QUEUED:	status_ticks = 40; break;
				case STATE_HOLD:	status_ticks = 20; break;

				case STATE_CYCLE:	status_ticks = 62; break;

				case STATE_HOMING:	status_ticks = 24; break;
				case STATE_ALARM:	status_ticks = 8; break;

				case STATE_CHECK_MODE:	status_ticks = 120; break;
				default: status_ticks = 2;
			}
			// toggle bit
			STATUS_LED_IN |= 1<<STATUS_LED_BIT;
		}

		TCNT2 = 1;
		TIFR2 = 0;
	}
};
