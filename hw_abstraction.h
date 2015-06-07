/*
 * hw_abstraction.h
 *
 *  Created on: 19.05.2015
 *      Author: legscmh
 */

#ifndef HW_ABSTRACTION_H_
#define HW_ABSTRACTION_H_

#include "config.h"

#ifdef CPU_MAP_TIVA
	#include <inc/tm4c123gh6zrb.h>

	#include <inc/hw_memmap.h>
	#include <inc/hw_gpio.h>
	#include <inc/hw_memmap.h>
	#include <inc/hw_types.h>
	#include <inc/hw_i2c.h>
//	#include <inc/hw_uart.h>
	#include <inc/hw_timer.h>
	#include <inc/hw_sysctl.h>
	#include <inc/hw_eeprom.h>

	#include <driverlib/gpio.h>
	#include <driverlib/i2c.h>
//	#include <driverlib/uart.h>
	#include <driverlib/interrupt.h>
	#include <driverlib/sysctl.h>
	#include <driverlib/timer.h>
	#include <driverlib/eeprom.h>
	#include <driverlib/fpu.h>
	#include <driverlib/systick.h>

	#include "usblib/usblib.h"
	#include "usblib/usbcdc.h"
	#include "usblib/usb-ids.h"
	#include "usblib/device/usbdevice.h"
	#include "usblib/device/usbdcdc.h"
	#include "usb_serial_structs.h"

	#define GLOBAL_INT_VECTOR(fname)		void fname()
	#define ISR_ROUTINE(vect,isr)			void isr()

	#define GPIO_OUTPUT_SET(port,mask)		GPIOPinTypeGPIOOutput( port, mask)
	#define GPIO_OUTPUT_STD(port,mask)		GPIOPadConfigSet( port, mask, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);

	#define GPIO_WRITE_MASKED(port,mask,v)	GPIOPinWrite( port, mask, v)

	#define GPIO_INPUT_SET(port,mask)		GPIOPinTypeGPIOInput( port, mask)
	#define GPIO_INPUT_STD(port,mask)		GPIOPadConfigSet( port, mask, 0, GPIO_PIN_TYPE_STD)
	#define GPIO_INPUT_INV(port,mask)		GPIOPadConfigSet( port, mask, 0, GPIO_PIN_TYPE_STD)

	#define GPIO_READ_MASKED(port,mask)		GPIOPinRead( port, mask)

	#define GPIO_ISR_SET(port,vect,fname)	GPIOIntRegister( port, fname)

	#define GPIO_INT_ENABLE(port,mask)		GPIOIntEnable( port, mask)
	#define GPIO_INT_DISABLE(port,mask)		GPIOIntDisable(port,mask)

	#define GPIO_INT_BOTH(port,mask)		GPIOIntTypeSet( port, mask, GPIO_BOTH_EDGES)

	#define GPIO_INT_CLEAR(port)			GPIOIntClear( port, GPIOIntStatus( port, true))

	#define GLOBAL_INT_ENABLE(flag)			/* globally enable int (flag) */
	#define GLOBAL_INT_DISABLE(flag)		/* globally enable int (flag) */

	// enable timer 1a in 32bit/wide mode
	#define ENABLE_PERIPHERAL(peri)			/* enable peripheral */\
											SysCtlGPIOAHBEnable( peri); \
											SysCtlPeripheralEnable( peri); \
											SysCtlDelay( 3)

	#define TIMER_GET_DELAY_HZ(hz)			((ui32SysClock / hz)/ 2)
	#define TIMER_SETUP( port, timer, reason, to)				/* setup periodic timer */ \
											TimerConfigure( port, TIMER_CFG_PERIODIC); \
											TimerLoadSet( port, timer, to); \
											TimerIntEnable(port, reason)
	#define TIMER_SET_DELAY( prt, tmr, dly)	TimerLoadSet( prt, tmr, dly)

	#define TIMER_ISR_SET(port,vect,fname)	TimerIntRegister(port, vect, fname)

	#define TIMER_INT_CLEAR(timer)			TimerIntClear( timer, TimerIntStatus( timer, true))

	// enable it
	#define TIMER_ENABLE( port, timer)		TimerEnable( port, timer)
	#define TIMER_DISABLE( port, timer)		TimerDisable( port, timer)

	#define M_PI							3.14159265359
	#define F_CPU							ui32SysClock

	//TODO add proper timing calculation here
	#define MSEC_TO_SYSCTL(ms)				( ms * 100000)
	#define _delay_ms(dly)					SysCtlDelay( dly * 10000)
	#define _delay_us(dly)					SysCtlDelay( dly * 10)
	#define pgm_read_byte_near(adr)			((uint8_t) *adr)

	#define PSTR( str)						str

#else

	#define GLOBAL_INT_VECTOR(fname)		/* isr: fname */
	#define ISR_ROUTINE(vect,isr)			ISR(vect)

	#define GPIO_OUTPUT_SET(port,mask)		port |= mask
	#define GPIO_OUTPUT_STD(port,mask)		/* std setup output */

	#define GPIO_INPUT_SET(port,mask)		port &= ~(mask)
	#define GPIO_INPUT_STD(port,mask)		port &= ~(mask)
	#define GPIO_INPUT_INV(port,mask)		port |= (mask)

	#define GPIO_READ_MASKED(port,mask)		(port & mask)

	#define GPIO_ISR_SET(port,vect,fname)	/* GPIOIntRegister(port,vect,fname) */
	#define GPIO_ISR_ROUTINE(vect,isr)		ISR(vect)

	#define GPIO_INT_ENABLE(port,mask)		port |= mask
	#define GPIO_INT_DISABLE(port,mask)		port &= ~mask

	#define GPIO_INT_BOTH(port,mask)		/* trigger on both edges */

	#define GPIO_INT_CLEAR(timer)			/* clear interrupt src */

	#define GLOBAL_INT_ENABLE(flag)			PCICR |= flag
	#define GLOBAL_INT_DISABLE(flag)		PCICR &= ~flag

	// enable timer 1a in 32bit/wide mode
	#define ENABLE_PERIPHERAL(peri)			/* enable peripheral */

	#define TIMER_GET_DELAY_HZ(hz)			(0)
	#define TIMER_SETUP(port,timer)				/* setup periodic timer */	\
											TCNT2	= 1;				\
											TIFR2   = 0x00;				\
											TIMSK2  = 1 << TOIE2;		\
											TCCR2A  = 0x00

	#define TIMER_ISR_SET(port,vect,fname)	/* TimerIntRegister(port,vect,fname) */

	#define TIMER_INT_CLEAR(timer)			/* clear/preset timer int flag */ \
											TCNT2 = 1; \
											TIFR2 = 0;


	// enable it
	#define TIMER_ENABLE( port, timer)		TCCR2B  = (1<<CS22) | (1<<CS21) | (1<<CS20)
	#define TIMER_DISABLE( port, timer)		TCCR2B = 0x00

	#define M_PI							3.14159265359

#endif

#endif /* HW_ABSTRACTION_H_ */
