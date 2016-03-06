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
#include "system.h"

#include "i2c_tiva.h"

//volatile uint8_t status_state;
//volatile uint8_t status_led;
volatile uint8_t status_ticks;
volatile uint8_t status_state = 0;

static uint32_t  g_ui32Colors[3];
static float g_fIntensity = 0.9f;

GLOBAL_INT_VECTOR( isrLedTimer);

#define RGB_PRESCALE 800
#define RGB_PERIOD 1000
#define RGB_TIMER_R_BASE	TIMER0_BASE
#define RGB_TIMER_GB_BASE	TIMER1_BASE

void statusRGBinit() {
    // Enable the GPIO Port and Timer for each LED
    //
    ENABLE_PERIPHERAL(SYSCTL_PERIPH_GPIOF);
    ENABLE_PERIPHERAL(SYSCTL_PERIPH_TIMER0);
    ENABLE_PERIPHERAL(SYSCTL_PERIPH_TIMER1);

    //
    // Reconfigure each LED's GPIO pad for timer control
    //
//    GPIOPadConfigSet( STATUS_LED_PORT, RED_GPIO_PIN | GREEN_GPIO_PIN | BLUE_GPIO_PIN, GPIO_STRENGTH_8MA_SC, GPIO_PIN_TYPE_STD);

	// switch pc6 to timer/pwm
    GPIOPinConfigure( GPIO_PF1_T0CCP1);
    GPIOPinConfigure( GPIO_PF2_T1CCP0);
    GPIOPinConfigure( GPIO_PF3_T1CCP1);
	GPIOPinTypeTimer( STATUS_LED_PORT, RED_GPIO_PIN | GREEN_GPIO_PIN | BLUE_GPIO_PIN);

	// setup 32bit timer
	TimerConfigure( RGB_TIMER_R_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_B_PWM);
	TimerConfigure( RGB_TIMER_GB_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM | TIMER_CFG_B_PWM);
/*
	TimerClockSourceSet( RGB_TIMER_R_BASE, TIMER_CLOCK_SYSTEM);
	TimerClockSourceSet( RGB_TIMER_GB_BASE, TIMER_CLOCK_SYSTEM);

	// tiemr is 32bit: we do not _need_ to prescale
	// set prescaler for 80MHz / 800 = 100kHz
	TimerPrescaleSet( RGB_TIMER_R_BASE, TIMER_B, RGB_PRESCALE);
	TimerPrescaleSet( RGB_TIMER_GB_BASE, TIMER_A, RGB_PRESCALE);
	TimerPrescaleSet( RGB_TIMER_GB_BASE, TIMER_B, RGB_PRESCALE);
*/
	//TimerPrescaleMatchSet( ESC_WTIMER_BASE, TIMER_A, PWM_PRESCALE);
	TimerLoadSet( RGB_TIMER_R_BASE, TIMER_B, RGB_PERIOD);
	TimerLoadSet( RGB_TIMER_GB_BASE, TIMER_A, RGB_PERIOD);
	TimerLoadSet( RGB_TIMER_GB_BASE, TIMER_B, RGB_PERIOD);

	TIMER0_CTL_R |= 0x4000;
	TIMER1_CTL_R |= 0x4000;
	TIMER1_CTL_R |= 0x40;

//	TimerControlLevel( WTIMER3_BASE, TIMER_A, true);

	// setup match for 50%
	TimerMatchSet( RGB_TIMER_R_BASE, TIMER_B, 950);
	TimerMatchSet( RGB_TIMER_GB_BASE, TIMER_A, 950);
	TimerMatchSet( RGB_TIMER_GB_BASE, TIMER_B, 950);

	// enable it
	TimerEnable( RGB_TIMER_R_BASE, TIMER_B);
	TimerEnable( RGB_TIMER_GB_BASE, TIMER_BOTH);
}

//*****************************************************************************
//
//! Set the output color.
//!
//! \param pui32RGBColor points to a three element array representing the
//! relative intensity of each color.  Red is element 0, Green is element 1,
//! Blue is element 2. 0x0000 is off.  0xFFFF is fully on.
//!
//! This function should be called by the application to set the color
//! of the RGB LED.
//!
//! \return None.
//
//*****************************************************************************
void statusRGBColorSet( uint32_t * pui32RGBColor) {
    uint32_t ui32Color[3];
    uint32_t ui32Index;

    for(ui32Index=0; ui32Index < 3; ui32Index++)
    {
        g_ui32Colors[ui32Index] = pui32RGBColor[ui32Index];
        ui32Color[ui32Index] = (uint32_t) (((float) pui32RGBColor[ui32Index]) * g_fIntensity + 0.5f);

        if(ui32Color[ui32Index] > 999)
        {
            ui32Color[ui32Index] = 999;
        }
    }

    TimerMatchSet(RGB_TIMER_R_BASE, TIMER_B, ui32Color[RED]);
    TimerMatchSet(RGB_TIMER_GB_BASE, TIMER_A, ui32Color[GREEN]);
    TimerMatchSet(RGB_TIMER_GB_BASE, TIMER_B, ui32Color[BLUE]);
}

//*****************************************************************************
//
//! Set the current output intensity.
//!
//! \param fIntensity is used to scale the intensity of all three colors by
//! the same amount.  fIntensity should be between 0.0 and 1.0.  This scale
//! factor is applied individually to all three colors.
//!
//! This function should be called by the application to set the intensity
//! of the RGB LED.
//!
//! \return None.
//
//*****************************************************************************
void statusRGBIntensitySet(float fIntensity) {
    g_fIntensity = fIntensity;
    statusRGBColorSet(g_ui32Colors);
}

void statusRGBSet(volatile uint32_t * pui32RGBColor,  float fIntensity) {
    statusRGBColorSet(pui32RGBColor);
    statusRGBIntensitySet(fIntensity);
}

void status_init()
{
	uint32_t col[3] = { 100, 100, 100};

	statusRGBinit();

	ENABLE_PERIPHERAL( STATUS_LED_PERI);
	ENABLE_PERIPHERAL( TIMER_LED_PERI);

	// set to output
//	GPIO_OUTPUT_SET( STATUS_LED_PORT, (1<<STATUS_LED_RED)|(1<<STATUS_LED_GREEN)|(1<<STATUS_LED_BLUE));	// STATUS_LED_DDR |= (1<<STATUS_LED_BIT);
//	GPIO_OUTPUT_STD( STATUS_LED_PORT, (1<<STATUS_LED_RED)|(1<<STATUS_LED_GREEN)|(1<<STATUS_LED_BLUE));

	//STATUS_LED_PORT |= 1<<STATUS_LED_BIT;

	TIMER_ISR_SET( TIMER_LED_PORT, TIMER_LED_VECT, isrLedTimer);

	TIMER_DISABLE( TIMER_LED_PORT, TIMER_A);
	TIMER_SETUP( TIMER_LED_PORT, TIMER_A, TIMER_TIMA_TIMEOUT, TIMER_GET_DELAY_HZ( 60));
	IntPrioritySet( INT_TIMER2A, IRQPRIO_LED);
	TIMER_ENABLE( TIMER_LED_PORT, TIMER_A);
/*
	TCCR2B = 0x00;			//Disable Timer2 while we set it up

	TCNT2	= 1;
	TIFR2   = 0x00;			//Timer2 INT Flag Reg: Clear Timer Overflow Flag
	TIMSK2  = 1 << TOIE2;	//Timer2 INT Reg: Timer2 Overflow Interrupt Enable

	TCCR2A  = 0x00;			//Timer2 Control Reg A: Normal port operation, Wave Gen Mode normal
	TCCR2B  = (1<<CS22) | (1<<CS21) | (1<<CS20);	//0x07;			//Timer2 Control Reg B: Timer Prescaler set to 1024
*/
//	status_state = PROGRAM_FLOW_COMPLETED;

	statusRGBColorSet( col);

	status_ticks = 62;
}

//Timer2 Overflow Interrupt Vector, called every 1ms
ISR_ROUTINE(TIMER_LED_VECT,isrLedTimer) {

	TIMER_INT_CLEAR( TIMER_LED_PORT);

	system_tick();
	
	status_ticks--;
	if ( status_ticks == 0) {

		if ( ! status_state) {
			switch (sys.state) {
				case STATE_IDLE:	status_ticks = 1; break;

				case STATE_QUEUED:	status_ticks = 20; break;
				case STATE_HOLD:	status_ticks = 40; break;

				case STATE_CYCLE:	status_ticks = 60; break;

				case STATE_HOMING:	status_ticks = 20; break;
				case STATE_ALARM:	status_ticks = 10; break;
				case STATE_CHECK_MODE:	status_ticks = 120; break;

				default: status_ticks = 235;
			}
			status_state = true;

			statusRGBIntensitySet( 0.6);
//			GPIO_WRITE_MASKED( STATUS_LED_PORT, 1<<STATUS_LED_RED, 1<<STATUS_LED_RED);	//STATUS_LED_PORT |= 1<<STATUS_LED_BIT;
		} else {
			switch (sys.state) {
				case STATE_IDLE:	status_ticks = 59; break;

				case STATE_QUEUED:	status_ticks = 40; break;
				case STATE_HOLD:	status_ticks = 20; break;

				case STATE_CYCLE:	status_ticks = 60; break;

				case STATE_HOMING:	status_ticks = 100; break;
				case STATE_ALARM:	status_ticks = 10; break;

				case STATE_CHECK_MODE:	status_ticks = 120; break;
				default: status_ticks = 5;
			}

			status_state = false;
			// toggle bit
			statusRGBIntensitySet( 0.0);
//			GPIO_WRITE_MASKED( STATUS_LED_PORT, 1<<STATUS_LED_RED, 0);	//STATUS_LED_IN |= 1<<STATUS_LED_BIT;
		}
	}
};
