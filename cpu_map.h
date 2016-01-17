/*
  cpu_map.h - CPU and pin mapping configuration file
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

/* The cpu_map.h file serves as a central pin mapping settings file for different processor
   types, i.e. AVR 328p or AVR Mega 2560. Grbl officially supports the Arduino Uno, but the 
   other supplied pin mappings are supplied by users, so your results may vary. */

// NOTE: This is still a work in progress. We are still centralizing the configurations to
// this file, so your success may vary for other CPUs.

#ifndef cpu_map_h
#define cpu_map_h

#include "hw_abstraction.h"

//----------------------------------------------------------------------------------------

#ifdef CPU_MAP_ATMEGA328P_TRADITIONAL // (Arduino Uno) Officially supported by Grbl.

  // Define serial port pins and interrupt vectors.
  #define SERIAL_RX     USART_RX_vect
  #define SERIAL_UDRE   USART_UDRE_vect

  // Define step pulse output pins. NOTE: All step bit pins must be on the same port.
  #define STEP_DDR        DDRD
  #define STEP_PORT       PORTD
  #define X_STEP_BIT      2  // Uno Digital Pin 2
  #define Y_STEP_BIT      4  // Uno Digital Pin 3
  #define Z_STEP_BIT      6  // Uno Digital Pin 4
  #define STEP_MASK       ((1<<X_STEP_BIT)|(1<<Y_STEP_BIT)|(1<<Z_STEP_BIT)) // All step bits

  #define STEP_TIMER_PERI	/* SYSCTL_PERIPH_TIMER1 */
  #define TIMER_STEP_PORT	/* TIMER1_BASE */
  #define TIMER_STEP_VECTA	TIMER1_COMPA_vect
  #define TIMER_STEP_VECTB	TIMER0_OVF_vect

  // Define step direction output pins. NOTE: All direction pins must be on the same port.
  #define DIRECTION_DDR     DDRD
  #define DIRECTION_PORT    PORTD
  #define X_DIRECTION_BIT   3  // Uno Digital Pin 5
  #define Y_DIRECTION_BIT   5  // Uno Digital Pin 6
  #define Z_DIRECTION_BIT   7  // Uno Digital Pin 7
  #define DIRECTION_MASK    ((1<<X_DIRECTION_BIT)|(1<<Y_DIRECTION_BIT)|(1<<Z_DIRECTION_BIT)) // All direction bits

  // Define stepper driver enable/disable output pin.
  #define STEPPERS_DISABLE_DDR    DDRB
  #define STEPPERS_DISABLE_PORT   PORTB
  #define STEPPERS_DISABLE_BIT    0  // Uno Digital Pin 8
  #define STEPPERS_DISABLE_MASK   (1<<STEPPERS_DISABLE_BIT)

  // Define homing/hard limit switch input pins and limit interrupt vectors. 
  // NOTE: All limit bit pins must be on the same port, but not on a port with other input pins (pinout).
  #define LIMIT_DDR        DDRB
  #define LIMIT_PIN        PINB
  #define LIMIT_PORT       PORTB
  #define X_LIMIT_BIT      1  // Uno Digital Pin 9
  #define Y_LIMIT_BIT      2  // Uno Digital Pin 10
#if ( SPINDLE_CTRL == CTRL_PIN)
  #ifdef VARIABLE_SPINDLE // Z Limit pin and spindle enabled swapped to access hardware PWM on Pin 11.  
    #define Z_LIMIT_BIT	   4 // Uno Digital Pin 12
  #else
    #define Z_LIMIT_BIT    3  // Uno Digital Pin 11
  #endif
#else
    #define Z_LIMIT_BIT    3  // Uno Digital Pin 11
#endif
  #define LIMIT_MASK       ((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT)|(1<<Z_LIMIT_BIT)) // All limit bits
  #define LIMIT_INT        PCIE0  // Pin change interrupt enable pin
  #define LIMIT_INT_vect   PCINT0_vect 
  #define LIMIT_PCMSK      PCMSK0 // Pin change interrupt register

  // status display LED
  #define TIMER_LED_PORT	//
  #define TIMER_LED_VECT	TIMER2_OVF_vect
  #define STATUS_LED_DDR   DDRB
  #define STATUS_LED_PORT  PORTB
  #define STATUS_LED_IN    PINB
  #define STATUS_LED_RED   5  // Uno Digital Pin 13 (NOTE: D13 can't be pulled-high input due to LED.)

#if ( SPINDLE_CTRL == CTRL_PIN)
  // Define spindle enable and spindle direction output pins.
  #define SPINDLE_ENABLE_DDR    DDRB
  #define SPINDLE_ENABLE_PORT   PORTB
  #ifdef VARIABLE_SPINDLE // Z Limit pin and spindle enabled swapped to access hardware PWM on Pin 11.  
    #define SPINDLE_ENABLE_BIT    3  // Uno Digital Pin 11
  #else
    #define SPINDLE_ENABLE_BIT    4  // Uno Digital Pin 12
  #endif  
  #define SPINDLE_DIRECTION_DDR   DDRB
  #define SPINDLE_DIRECTION_PORT  PORTB
  #define SPINDLE_DIRECTION_BIT   5  // Uno Digital Pin 13 (NOTE: D13 can't be pulled-high input due to LED.)
#endif

#if ( COOLANT_CTRL == CTRL_PIN)
  // Define flood and mist coolant enable output pins.
  // NOTE: Uno analog pins 4 and 5 are reserved for an i2c interface, and may be installed at
  // a later date if flash and memory space allows.
  #define COOLANT_FLOOD_DDR   DDRC
  #define COOLANT_FLOOD_PORT  PORTC
  #define COOLANT_FLOOD_BIT   3  // Uno Analog Pin 3
  #ifdef ENABLE_M7 // Mist coolant disabled by default. See config.h to enable/disable.
    #define COOLANT_MIST_DDR   DDRC
    #define COOLANT_MIST_PORT  PORTC
    #define COOLANT_MIST_BIT   4 // Uno Analog Pin 4
  #endif  
#endif

  // Define user-control pinouts (cycle start, reset, feed hold) input pins.
  // NOTE: All pinouts pins must be on the same port and not on a port with other input pins (limits).
  #define PINOUT_DDR       DDRC
  #define PINOUT_PIN       PINC
  #define PINOUT_PORT      PORTC
  #define PIN_RESET        1  // Uno Analog Pin 0
  #define PIN_FEED_HOLD    0  // Uno Analog Pin 1
  #define PIN_CYCLE_START  2  // Uno Analog Pin 2
  #define PINOUT_INT       PCIE1  // Pin change interrupt enable pin
  #define PINOUT_INT_vect  PCINT1_vect
  #define PINOUT_PCMSK     PCMSK1 // Pin change interrupt register
  #define PINOUT_MASK ((1<<PIN_RESET)|(1<<PIN_FEED_HOLD)|(1<<PIN_CYCLE_START))
  
  // Define probe switch input pin.
  #define PROBE_DDR       DDRB
  #define PROBE_PIN       PINB
  #define PROBE_PORT      PORTB
  #define PROBE_BIT       4  // Uno Digital Pin 12
  #define PROBE_MASK      (1<<PROBE_BIT)

  #define READY_DDR       DDRC
  #define READY_PIN       PINC
  #define READY_PORT      PORTC
  #define PIN_READY       3  // Uno Analog Pin 3

#if ( SPINDLE_CTRL == CTRL_PIN)
  #ifdef VARIABLE_SPINDLE
    // Advanced Configuration Below You should not need to touch these variables
    #define TCCRA_REGISTER	 TCCR2A
    #define TCCRB_REGISTER	 TCCR2B
    #define OCR_REGISTER     OCR2A

    #define COMB_BIT	     COM2A1
    #define WAVE0_REGISTER	 WGM20
    #define WAVE1_REGISTER	 WGM21
    #define WAVE2_REGISTER	 WGM22
    #define WAVE3_REGISTER	 WGM23

    // NOTE: On the 328p, these must be the same as the SPINDLE_ENABLE settings.
    #define SPINDLE_PWM_DDR	  SPINDLE_ENABLE_DDR
    #define SPINDLE_PWM_PORT  SPINDLE_ENABLE_PORT
    #define SPINDLE_PWM_BIT	  SPINDLE_ENABLE_BIT // Shared with SPINDLE_ENABLE.
  #endif // End of VARIABLE_SPINDLE
#endif
#endif

//----------------------------------------------------------------------------------------

#ifdef CPU_MAP_TIVA // (Arduino Uno) Officially supported by Grbl.

	// Define serial port pins and interrupt vectors.
	#define SERIAL_RX				USART_RX_vect
	#define SERIAL_UDRE				USART_UDRE_vect

	// serial port definitions
	#define TIVA_SERIAL_UART_PERI	SYSCTL_PERIPH_UART0
	#define TIVA_SERIAL_GPIO_PERI	SYSCTL_PERIPH_GPIOA

	#define TIVA_SERIAL_UART		UART0_BASE
	#define TIVA_SERIAL_PORT		GPIO_PORTA_AHB_BASE
	#define TIVA_SERIAL_RX_PIN		GPIO_PA0_U0RX
    #define TIVA_SERIAL_TX_PIN		GPIO_PA1_U0TX

	#define TIVA_SERIAL_PINS		(GPIO_PIN_0 | GPIO_PIN_1)

	// Define step pulse output pins. NOTE: All step bit pins must be on the same port.
	#define STEP_PERI				SYSCTL_PERIPH_GPIOA
	#define STEP_DDR				GPIO_PORTA_AHB_BASE
	#define STEP_PORT				GPIO_PORTA_AHB_BASE
	#define X_STEP_BIT				2  // Uno Digital Pin 2
	#define Y_STEP_BIT				3  // Uno Digital Pin 3
	#define Z_STEP_BIT				4  // Uno Digital Pin 4
	#define A_STEP_BIT				5  // Uno Digital Pin 2
	#define B_STEP_BIT				6  // Uno Digital Pin 3
	#define C_STEP_BIT				7  // Uno Digital Pin 4
	#define STEP_MASK   		    ((1<<X_STEP_BIT)|(1<<Y_STEP_BIT)|(1<<Z_STEP_BIT)|(1<<A_STEP_BIT)|(1<<B_STEP_BIT)|(1<<C_STEP_BIT)) // All step bits

	#define STEP_TIMER_PERI			SYSCTL_PERIPH_WTIMER0
	#define TIMER_STEP_PORT			WTIMER0_BASE
	#define TIMER_STEP_PERIOD		TIMER_A
	#define TIMER_STEP_PULSE		TIMER_B

	// Define step direction output pins. NOTE: All direction pins must be on the same port.
	#define DIRECTION_PERI			SYSCTL_PERIPH_GPIOE
	#define DIRECTION_DDR			GPIO_PORTE_AHB_BASE
	#define DIRECTION_PORT			GPIO_PORTE_AHB_BASE
	#define X_DIRECTION_BIT			0  // Uno Digital Pin 5
	#define Y_DIRECTION_BIT			1  // Uno Digital Pin 6
	#define Z_DIRECTION_BIT			2  // Uno Digital Pin 7
	#define A_DIRECTION_BIT			3  // Uno Digital Pin 5
	#define B_DIRECTION_BIT			4  // Uno Digital Pin 6
	#define C_DIRECTION_BIT			5  // Uno Digital Pin 7
	#define DIRECTION_MASK		((1<<X_DIRECTION_BIT)|(1<<Y_DIRECTION_BIT)|(1<<Z_DIRECTION_BIT)|(1<<A_DIRECTION_BIT)|(1<<B_DIRECTION_BIT)|(1<<C_DIRECTION_BIT)) // All direction bits

	// Define stepper driver enable/disable output pin.
	#define STEPPERS_DISABLE_PERI		SYSCTL_PERIPH_GPIOC
	#define STEPPERS_DISABLE_DDR	GPIO_PORTC_AHB_BASE
	#define STEPPERS_DISABLE_PORT	GPIO_PORTC_AHB_BASE
	#define STEPPERS_DISABLE_BIT	4  // Uno Digital Pin 8
	#define STEPPERS_DISABLE_MASK	(1<<STEPPERS_DISABLE_BIT)

	// Define homing/hard limit switch input pins and limit interrupt vectors.
	// NOTE: All limit bit pins must be on the same port, but not on a port with other input pins (pinout).
	#define LIMIT_PERI		SYSCTL_PERIPH_GPIOB
	#define LIMIT_DDR		GPIO_PORTB_AHB_BASE
	#define LIMIT_PIN		GPIO_PORTB_AHB_BASE
	#define LIMIT_PORT		GPIO_PORTB_AHB_BASE
	#define X_LIMIT_BIT		0  // Uno Digital Pin 9
	#define Y_LIMIT_BIT		1  // Uno Digital Pin 10
	#define Z_LIMIT_BIT		4 // Uno Digital Pin 12
	#define A_LIMIT_BIT		5  // Uno Digital Pin 9
	#define B_LIMIT_BIT		6  // Uno Digital Pin 10
	#define C_LIMIT_BIT		7  // Uno Digital Pin 9
	#define LIMIT_MASK		((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT)|(1<<Z_LIMIT_BIT)|(1<<A_LIMIT_BIT)|(1<<B_LIMIT_BIT)|(1<<C_LIMIT_BIT)) // All limit bits
	#define LIMIT_INT		PCIE0  // Pin change interrupt enable pin
	#define LIMIT_INT_vect	PCINT0_vect
	#define LIMIT_PCMSK		PCMSK0 // Pin change interrupt register

	// status display LED
	#define TIMER_LED_PERI	SYSCTL_PERIPH_TIMER0
	#define TIMER_LED_PORT	TIMER0_BASE
	#define TIMER_LED_VECT	TIMER_A

	#define STATUS_LED_PERI	SYSCTL_PERIPH_GPIOF
	#define STATUS_LED_DDR	GPIO_PORTF_AHB_BASE
	#define STATUS_LED_PORT	GPIO_PORTF_AHB_BASE
	#define STATUS_LED_IN	GPIO_PORTF_AHB_BASE
	#define STATUS_LED_RED	1	// onboard LED
	#define STATUS_LED_BLUE	2	// onboard LED
	#define STATUS_LED_GREEN	3  // onboard LED

	// Define user-control pinouts (cycle start, reset, feed hold) input pins.
	// NOTE: All pinouts pins must be on the same port and not on a port with other input pins (limits).
	#define PINOUT_PERI		SYSCTL_PERIPH_GPIOF
	#define PINOUT_DDR		GPIO_PORTF_AHB_BASE
	#define PINOUT_PIN		GPIO_PORTF_AHB_BASE
	#define PINOUT_PORT		GPIO_PORTF_AHB_BASE
	#define PIN_RESET		5  // Uno Analog Pin 0
	#define PIN_FEED_HOLD	0  // Uno Analog Pin 1
	#define PIN_CYCLE_START	4  // Uno Analog Pin 2
	#define PINOUT_INT		PCIE1  // Pin change interrupt enable pin
	#define PINOUT_INT_vect	PCINT1_vect
	#define PINOUT_PCMSK	PCMSK1 // Pin change interrupt register
	#define PINOUT_MASK		((1<<PIN_FEED_HOLD)|(1<<PIN_CYCLE_START))	//(1<<PIN_RESET)|

	// Define probe switch input pin.
	#define PROBE_PERI		SYSCTL_PERIPH_GPIOC
	#define PROBE_DDR		GPIO_PORTC_AHB_BASE
	#define PROBE_PIN		GPIO_PORTC_AHB_BASE
	#define PROBE_PORT		GPIO_PORTC_AHB_BASE
	#define PROBE_BIT		7  // Uno Digital Pin 12
	#define PROBE_MASK		(1<<PROBE_BIT)

	#define ESC_PERI		SYSCTL_PERIPH_GPIOC
	#define ESC_PORT		GPIO_PORTC_AHB_BASE
	#define ESC_PIN			GPIO_PORTC_AHB_BASE
	#define ESC_PORT		GPIO_PORTC_AHB_BASE
	#define ESC_BIT			6
	#define ESC_MASK		(1<<ESC_BIT)

	#define ESC_WTIMER_PERI	SYSCTL_PERIPH_WTIMER1
	#define ESC_WTIMER_BASE	WTIMER1_BASE
	#define ESC_PIN_PWM		WT1CCP0

	#define TIVA_I2C_PERI	SYSCTL_PERIPH_I2C0
	#define TIVA_I2C_BASE	I2C0_BASE
	#define TIVA_I2C_PINPERI	SYSCTL_PERIPH_GPIOB
	#define TIVA_I2C_PINPORT	GPIO_PORTB_AHB_BASE
	#define TIVA_I2C_SCL	GPIO_PB2_I2C0SCL
    #define TIVA_I2C_SDA	GPIO_PB3_I2C0SDA
	#define TIVA_I2C_PINS	(GPIO_PIN_2 | GPIO_PIN_3)

	#define I2CINT_PERI		SYSCTL_PERIPH_GPIOC
	#define I2CINT_DDR		GPIO_PORTC_AHB_BASE
	#define I2CINT_PIN		GPIO_PORTC_AHB_BASE
	#define I2CINT_PORT		GPIO_PORTC_AHB_BASE
	#define I2CINT_BIT		5  // Uno Analog Pin 3
	#define I2CINT_MASK		(1<<I2CINT_BIT)
	#define I2CINT_vect		INT_I2C0

	#define SPI_GPIO_PERI	SYSCTL_PERIPH_GPIOD
	#define SPI_PERI		SYSCTL_PERIPH_SSI1
	#define SPI_PORT		SSI1_BASE
	#define SPI_GPIO_PORT	GPIO_PORTD_AHB_BASE
	#define SPI_MASK		((1<<0)|(1<<1)|(1<<2)|(1<<3))

	#define QEI_GPIO_PERI	SYSCTL_PERIPH_GPIOD
	#define QEI_PERI		SYSCTL_PERIPH_QEI0
	#define QEI_PORT		QEI0_BASE
	#define QEI_GPIO_PORT	GPIO_PORTD_AHB_BASE
	#define QEI_MASK		((1<<6)|(1<<7))

	// priorities for interrupts
	#define IRQPRIO_MOTION		(0 << 5)
	#define IRQPRIO_MPULSE		(0 << 5)
	#define IRQPRIO_LIMIT		(1 << 5)
	#define IRQPRIO_FAULT		(1 << 5)
	#define IRQPRIO_SERIAL		(2 << 5)
	#define IRQPRIO_I2C			(2 << 5)
	#define IRQPRIO_SSI			(3 << 5)
	#define IRQPRIO_QEI			(4 << 5)
	#define IRQPRIO_LED			(5 << 5)

#endif

//----------------------------------------------------------------------------------------

#ifdef CPU_MAP_ATMEGA328P_NEW // (Arduino Uno) New test pinout configuration. Still subject to change. Not finalized!

  // Define serial port pins and interrupt vectors.
  #define SERIAL_RX     USART_RX_vect
  #define SERIAL_UDRE   USART_UDRE_vect

  // Define step pulse output pins. NOTE: All step bit pins must be on the same port.
  #define STEP_DDR        DDRD
  #define STEP_PORT       PORTD
  #define X_STEP_BIT      2  // Uno Digital Pin 2
  #define Y_STEP_BIT      3  // Uno Digital Pin 3
  #define Z_STEP_BIT      4  // Uno Digital Pin 4
  #define STEP_MASK       ((1<<X_STEP_BIT)|(1<<Y_STEP_BIT)|(1<<Z_STEP_BIT)) // All step bits

  // Define step direction output pins. NOTE: All direction pins must be on the same port.
  #define DIRECTION_DDR     DDRD
  #define DIRECTION_PORT    PORTD
  #define X_DIRECTION_BIT   5  // Uno Digital Pin 5
  #define Y_DIRECTION_BIT   6  // Uno Digital Pin 6
  #define Z_DIRECTION_BIT   7  // Uno Digital Pin 7
  #define DIRECTION_MASK    ((1<<X_DIRECTION_BIT)|(1<<Y_DIRECTION_BIT)|(1<<Z_DIRECTION_BIT)) // All direction bits

  // Define stepper driver enable/disable output pin.
  #define STEPPERS_DISABLE_DDR    DDRB
  #define STEPPERS_DISABLE_PORT   PORTB
  #define STEPPERS_DISABLE_BIT    0  // Uno Digital Pin 8
  #define STEPPERS_DISABLE_MASK   (1<<STEPPERS_DISABLE_BIT)

  // Define homing/hard limit switch input pins and limit interrupt vectors. 
  // NOTE: All limit bit pins must be on the same port, but not on a port with other input pins (pinout).
  #define LIMIT_DDR        DDRB
  #define LIMIT_PIN        PINB
  #define LIMIT_PORT       PORTB
  #define X_LIMIT_BIT      1  // Uno Digital Pin 9
  #define Y_LIMIT_BIT      2  // Uno Digital Pin 10
  #ifdef VARIABLE_SPINDLE // Z Limit pin and spindle enabled swapped to access hardware PWM on Pin 11.  
    #define Z_LIMIT_BIT	   4 // Uno Digital Pin 12
  #else
    #define Z_LIMIT_BIT    3  // Uno Digital Pin 11
  #endif
  #define LIMIT_MASK       ((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT)|(1<<Z_LIMIT_BIT)) // All limit bits
  #define LIMIT_INT        PCIE0  // Pin change interrupt enable pin
  #define LIMIT_INT_vect   PCINT0_vect 
  #define LIMIT_PCMSK      PCMSK0 // Pin change interrupt register

  // Define spindle enable and spindle direction output pins.
  #define SPINDLE_ENABLE_DDR    DDRB
  #define SPINDLE_ENABLE_PORT   PORTB
  #ifdef VARIABLE_SPINDLE // Z Limit pin and spindle enabled swapped to access hardware PWM on Pin 11.  
    #define SPINDLE_ENABLE_BIT    3  // Uno Digital Pin 11
  #else
    #define SPINDLE_ENABLE_BIT    4  // Uno Digital Pin 12
  #endif  
  #define SPINDLE_DIRECTION_DDR   DDRB
  #define SPINDLE_DIRECTION_PORT  PORTB
  #define SPINDLE_DIRECTION_BIT   5  // Uno Digital Pin 13 (NOTE: D13 can't be pulled-high input due to LED.)

  // Define flood and mist coolant enable output pins.
  // NOTE: Uno analog pins 4 and 5 are reserved for an i2c interface, and may be installed at
  // a later date if flash and memory space allows.
  #define COOLANT_FLOOD_DDR   DDRC
  #define COOLANT_FLOOD_PORT  PORTC
  #define COOLANT_FLOOD_BIT   4  // Uno Analog Pin 3
  #ifdef ENABLE_M7 // Mist coolant disabled by default. See config.h to enable/disable.
    #define COOLANT_MIST_DDR   DDRC
    #define COOLANT_MIST_PORT  PORTC
    #define COOLANT_MIST_BIT   5 // Uno Analog Pin 4
  #endif  

  // Define user-control pinouts (cycle start, reset, feed hold) input pins.
  // NOTE: All pinouts pins must be on the same port and not on a port with other input pins (limits).
  #define PINOUT_DDR       DDRC
  #define PINOUT_PIN       PINC
  #define PINOUT_PORT      PORTC
  #define PIN_RESET        1  // Uno Analog Pin 1
  #define PIN_FEED_HOLD    2  // Uno Analog Pin 2
  #define PIN_CYCLE_START  3  // Uno Analog Pin 3
  #define PINOUT_INT       PCIE1  // Pin change interrupt enable pin
  #define PINOUT_INT_vect  PCINT1_vect
  #define PINOUT_PCMSK     PCMSK1 // Pin change interrupt register
  #define PINOUT_MASK ((1<<PIN_RESET)|(1<<PIN_FEED_HOLD)|(1<<PIN_CYCLE_START))
  
  // Define probe switch input pin.
  #define PROBE_DDR       DDRC
  #define PROBE_PIN       PINC
  #define PROBE_PORT      PORTC
  #define PROBE_BIT       0  // Uno Analog Pin 0
  #define PROBE_MASK      (1<<PROBE_BIT)

  
  #ifdef VARIABLE_SPINDLE
    // Advanced Configuration Below You should not need to touch these variables
    #define TCCRA_REGISTER	 TCCR2A
    #define TCCRB_REGISTER	 TCCR2B
    #define OCR_REGISTER     OCR2A

    #define COMB_BIT	     COM2A1
    #define WAVE0_REGISTER	 WGM20
    #define WAVE1_REGISTER	 WGM21
    #define WAVE2_REGISTER	 WGM22
    #define WAVE3_REGISTER	 WGM23

    // NOTE: On the 328p, these must be the same as the SPINDLE_ENABLE settings.
    #define SPINDLE_PWM_DDR	  SPINDLE_ENABLE_DDR
    #define SPINDLE_PWM_PORT  SPINDLE_ENABLE_PORT
    #define SPINDLE_PWM_BIT	  SPINDLE_ENABLE_BIT // Shared with SPINDLE_ENABLE.
  #endif // End of VARIABLE_SPINDLE

#endif

//----------------------------------------------------------------------------------------

#ifdef CPU_MAP_ATMEGA2560 // (Arduino Mega 2560) Working @EliteEng

  // Serial port pins
  #define SERIAL_RX USART0_RX_vect
  #define SERIAL_UDRE USART0_UDRE_vect

  // Increase Buffers to make use of extra SRAM
  //#define RX_BUFFER_SIZE		256
  //#define TX_BUFFER_SIZE		128
  //#define BLOCK_BUFFER_SIZE	36
  //#define LINE_BUFFER_SIZE	100

  // Define step pulse output pins. NOTE: All step bit pins must be on the same port.
  #define STEP_DDR      DDRA
  #define STEP_PORT     PORTA
  #define STEP_PIN      PINA
  #define X_STEP_BIT        2 // MEGA2560 Digital Pin 24
  #define Y_STEP_BIT        3 // MEGA2560 Digital Pin 25
  #define Z_STEP_BIT        4 // MEGA2560 Digital Pin 26
  #define STEP_MASK ((1<<X_STEP_BIT)|(1<<Y_STEP_BIT)|(1<<Z_STEP_BIT)) // All step bits

  // Define step direction output pins. NOTE: All direction pins must be on the same port.
  #define DIRECTION_DDR      DDRA
  #define DIRECTION_PORT     PORTA
  #define DIRECTION_PIN      PINA
  #define X_DIRECTION_BIT   5 // MEGA2560 Digital Pin 27
  #define Y_DIRECTION_BIT   6 // MEGA2560 Digital Pin 28
  #define Z_DIRECTION_BIT   7 // MEGA2560 Digital Pin 29
  #define DIRECTION_MASK ((1<<X_DIRECTION_BIT)|(1<<Y_DIRECTION_BIT)|(1<<Z_DIRECTION_BIT)) // All direction bits

  // Define stepper driver enable/disable output pin.
  #define STEPPERS_DISABLE_DDR   DDRB
  #define STEPPERS_DISABLE_PORT  PORTB
  #define STEPPERS_DISABLE_BIT   7 // MEGA2560 Digital Pin 13
  #define STEPPERS_DISABLE_MASK (1<<STEPPERS_DISABLE_BIT)

  // NOTE: All limit bit pins must be on the same port
  #define LIMIT_DDR       DDRB
  #define LIMIT_PORT      PORTB
  #define LIMIT_PIN       PINB
  #define X_LIMIT_BIT     4 // MEGA2560 Digital Pin 10
  #define Y_LIMIT_BIT     5 // MEGA2560 Digital Pin 11
  #define Z_LIMIT_BIT     6 // MEGA2560 Digital Pin 12
  #define LIMIT_INT       PCIE0  // Pin change interrupt enable pin
  #define LIMIT_INT_vect  PCINT0_vect 
  #define LIMIT_PCMSK     PCMSK0 // Pin change interrupt register
  #define LIMIT_MASK ((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT)|(1<<Z_LIMIT_BIT)) // All limit bits

  // Define spindle enable and spindle direction output pins.
  #define SPINDLE_ENABLE_DDR   DDRH
  #define SPINDLE_ENABLE_PORT  PORTH
  #define SPINDLE_ENABLE_BIT   3 // MEGA2560 Digital Pin 6
  #define SPINDLE_DIRECTION_DDR   DDRE
  #define SPINDLE_DIRECTION_PORT  PORTE
  #define SPINDLE_DIRECTION_BIT   3 // MEGA2560 Digital Pin 5

  // Define flood and mist coolant enable output pins.
  // NOTE: Uno analog pins 4 and 5 are reserved for an i2c interface, and may be installed at
  // a later date if flash and memory space allows.
  #define COOLANT_FLOOD_DDR   DDRH
  #define COOLANT_FLOOD_PORT  PORTH
  #define COOLANT_FLOOD_BIT   5 // MEGA2560 Digital Pin 8
  #ifdef ENABLE_M7 // Mist coolant disabled by default. See config.h to enable/disable.
    #define COOLANT_MIST_DDR   DDRH
    #define COOLANT_MIST_PORT  PORTH
    #define COOLANT_MIST_BIT   6 // MEGA2560 Digital Pin 9
  #endif  

  // Define user-control pinouts (cycle start, reset, feed hold) input pins.
  // NOTE: All pinouts pins must be on the same port and not on a port with other input pins (limits).
  #define PINOUT_DDR       DDRK
  #define PINOUT_PIN       PINK
  #define PINOUT_PORT      PORTK
  #define PIN_RESET        0  // MEGA2560 Analog Pin 8
  #define PIN_FEED_HOLD    1  // MEGA2560 Analog Pin 9
  #define PIN_CYCLE_START  2  // MEGA2560 Analog Pin 10
  #define PINOUT_INT       PCIE2  // Pin change interrupt enable pin
  #define PINOUT_INT_vect  PCINT2_vect
  #define PINOUT_PCMSK     PCMSK2 // Pin change interrupt register
  #define PINOUT_MASK ((1<<PIN_RESET)|(1<<PIN_FEED_HOLD)|(1<<PIN_CYCLE_START))

  // Define probe switch input pin.
  #define PROBE_DDR       DDRK
  #define PROBE_PIN       PINK
  #define PROBE_PORT      PORTK
  #define PROBE_BIT       3  // MEGA2560 Analog Pin 11
  #define PROBE_MASK      (1<<PROBE_BIT)

  // Start of PWM & Stepper Enabled Spindle
  #ifdef VARIABLE_SPINDLE
    // Advanced Configuration Below You should not need to touch these variables
    // Set Timer up to use TIMER2B which is attached to Digital Pin 9
    #define TCCRA_REGISTER		TCCR2A
    #define TCCRB_REGISTER		TCCR2B
    #define OCR_REGISTER		OCR2B

    #define COMB_BIT			COM2B1
    #define WAVE0_REGISTER		WGM20
    #define WAVE1_REGISTER		WGM21
    #define WAVE2_REGISTER		WGM22
    #define WAVE3_REGISTER		WGM23

    #define SPINDLE_PWM_DDR		DDRH
    #define SPINDLE_PWM_PORT    PORTH
    #define SPINDLE_PWM_BIT		6 // MEGA2560 Digital Pin 9
  #endif // End of VARIABLE_SPINDLE

#endif

//----------------------------------------------------------------------------------------

/* 
#ifdef CPU_MAP_CUSTOM_PROC
  // For a custom pin map or different processor, copy and paste one of the default cpu map
  // settings above and modify it to your needs. Then, make sure the defined name is also
  // changed in the config.h file.
#endif
*/

#endif
