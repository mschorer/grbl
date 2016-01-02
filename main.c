/*
  main.c - An embedded CNC Controller with rs274/ngc (g-code) support
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
    Copyright (c) 2011-2012 Sungeun K. Jeon
*/  

#define TARGET_IS_TM4C123_RB1

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
/*
//#include <inc/tm4c123gh6zrb.h>

#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <inc/hw_ssi.h>
#include <inc/hw_i2c.h>
#include <inc/hw_gpio.h>
#include <inc/hw_adc.h>
#include <inc/hw_timer.h>
// #include <inc/hw_ints.h>
#include <inc/hw_sysctl.h>

//#include <driverlib/rom.h>
//#include <driverlib/rom_map.h>

#include <driverlib/pin_map.h>
#include <driverlib/sysctl.h>
#include <driverlib/gpio.h>

#include <driverlib/ssi.h>
#include <driverlib/i2c.h>
#include <driverlib/interrupt.h>
#include <driverlib/sysctl.h>

#include <driverlib/adc.h>
#include <driverlib/timer.h>

#include <assert.h>
*/
#include "hw_abstraction.h"

#include "system.h"
#include "serial.h"
#include "settings.h"
#include "protocol.h"
#include "gcode.h"
#include "planner.h"
#include "stepper.h"
#include "machine_control.h"
#include "spindle_control.h"
#include "coolant_control.h"
#include "motion_control.h"

#include "io_mcp23008.h"
#include "io_other.h"

#include "limits.h"
#include "probe.h"
#include "report.h"
#include "status.h"
#include "eeprom.h"

// Declare system global variable structure
system_t sys; 

int main(void)
{
	// Initialize system upon power-up.

	FPULazyStackingEnable();
	FPUEnable();

	SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

	status_init();

	eeprom_init();

  serial_init();   // Setup serial baud rate and interrupts
  settings_init(); // Load grbl settings from EEPROM
  stepper_init();  // Configure stepper pins and interrupt timers
  system_init();   // Configure pinout pins and pin-change interrupt

  io_init();
  io_other_init();
  mctrl_init();

  memset(&sys, 0, sizeof(sys));  // Clear all system variables
  sys.abort = true;   // Set abort to complete initialization

#ifdef CPU_MAP_TIVA
  IntMasterEnable();
#else
  sei(); // Enable interrupts
#endif

  // Check for power-up and set system alarm if homing is enabled to force homing cycle
  // by setting Grbl's alarm state. Alarm locks out all g-code commands, including the
  // startup scripts, but allows access to settings and internal commands. Only a homing
  // cycle '$H' or kill alarm locks '$X' will disable the alarm.
  // NOTE: The startup script will run after successful completion of the homing cycle, but
  // not after disabling the alarm locks. Prevents motion startup blocks from crashing into
  // things uncontrollably. Very bad.
  #ifdef HOMING_INIT_LOCK
    if (bit_istrue(settings.flags,BITFLAG_HOMING_ENABLE)) { sys.state = STATE_ALARM; }
  #endif
  
  // Grbl initialization loop upon power-up or a system abort. For the latter, all processes
  // will return to this loop to be cleanly re-initialized.
  for(;;) {

    // TODO: Separate configure task that require interrupts to be disabled, especially upon
    // a system abort and ensuring any active interrupts are cleanly reset.
  
    // Reset Grbl primary systems.
    serial_reset_read_buffer(); // Clear serial read buffer
    gc_init(); // Set g-code parser to default state
	
    spindle_init();
    coolant_init();
    tools_init();
	mctrl_flush();
	
    limits_init(); 
    probe_init();
    plan_reset(); // Clear block buffer and planner variables
    st_reset(); // Clear stepper subsystem variables.

    // Sync cleared gcode and planner positions to current system position.
    plan_sync_position();
    gc_sync_position();

    // Reset system variables.
    sys.abort = false;
    sys.execute = 0;
    if (bit_istrue(settings.flags,BITFLAG_AUTO_START)) { sys.auto_start = true; }
    else { sys.auto_start = false; }
          
    // Start Grbl main loop. Processes program inputs and executes them.
    protocol_main_loop();
    
  }
}
