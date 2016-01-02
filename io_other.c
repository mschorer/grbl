/*
 * io_other.c
 *
 *  Created on: 17.12.2015
 *      Author: markus
 */

#include "cpu_map.h"

#ifdef CPU_MAP_TIVA

#include <inc/tm4c123gh6pm.h>

#include <inc/hw_memmap.h>
#include <inc/hw_gpio.h>
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <inc/hw_qei.h>
#include <inc/hw_ssi.h>
#include <inc/hw_timer.h>
#include <inc/hw_sysctl.h>

#include <driverlib/pin_map.h>

#include <driverlib/gpio.h>
#include <driverlib/ssi.h>
#include <driverlib/qei.h>
#include <driverlib/interrupt.h>
#include <driverlib/sysctl.h>
#include <driverlib/timer.h>

#include "cpu_map.h"
#include "io_other.h"

// Inverts the probe pin state depending on user settings.
uint8_t probe_invert_mask;

// 80MHz, prescale with 80
#define PWM_PRESCALE	80				// prescale by 80, base clock is 1MHz
#define PWM_PERIOD		20000			// 20ms
#define PWM_BASE		1000			// 1ms
#define PWM_MAX			1000			// 1ms

volatile uint32_t rpmTargetValue = 0;
volatile uint32_t controlValue = 0;

//-----------------------------------------------------------------

void isrSSI1() {
	unsigned long ulStatus;

	ulStatus = SSIIntStatus( SPI_PORT, true);
	SSIIntClear( SPI_PORT, ulStatus);
}

void isrQEI0() {
	unsigned long ulStatus;

	ulStatus = QEIIntStatus( QEI_PORT, true);
	QEIIntClear( QEI_PORT, ulStatus);

	// add closed loop control here
	uint32_t currentRpm = QEIVelocityGet( QEI_GPIO_PORT);
	if ( rpmTargetValue != currentRpm) {

		// adjust pwm value
		setPWM( controlValue);
	}
}

//-----------------------------------------------------------------

// Probe pin initialization routine.
void io_other_init()
{
	uint32_t rx;

	//-----------------------------------------------------------------
	// set up esc

	ENABLE_PERIPHERAL( ESC_PERI);
//	GPIO_OUTPUT_SET( ESC_PORT, ESC_MASK);

	// enable timer 3 a in 32bit/wide mode
	ENABLE_PERIPHERAL( ESC_WTIMER_PERI);

	// switch pc6 to timer/pwm
    GPIOPinConfigure( GPIO_PC6_WT1CCP0);
	GPIOPinTypeTimer( ESC_PORT, ESC_MASK);

	// setup 32bit timer
	TimerConfigure( ESC_WTIMER_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM);
	TimerClockSourceSet( ESC_WTIMER_BASE, TIMER_CLOCK_SYSTEM);

	// tiemr is 32bit: we do not _need_ to prescale
	// set prescaler for 80MHz / 800 = 100kHz
	TimerPrescaleSet( ESC_WTIMER_BASE, TIMER_A, PWM_PRESCALE);

	//TimerPrescaleMatchSet( ESC_WTIMER_BASE, TIMER_A, PWM_PRESCALE);
	TimerLoadSet( ESC_WTIMER_BASE, TIMER_A, PWM_PERIOD);

//	TimerControlLevel( WTIMER3_BASE, TIMER_A, true);

	// setup match for 50%
	setPWM( 0);

	// enable it
	TimerEnable( ESC_WTIMER_BASE, TIMER_A);

	//-----------------------------------------------------------------
	// SSI master setup

	ENABLE_PERIPHERAL( SPI_PERI);
	ENABLE_PERIPHERAL( SPI_GPIO_PERI);

    GPIOPinConfigure(GPIO_PD0_SSI1CLK);
    GPIOPinConfigure(GPIO_PD1_SSI1FSS);
    GPIOPinConfigure(GPIO_PD2_SSI1RX);
    GPIOPinConfigure(GPIO_PD3_SSI1TX);

    GPIOPinTypeSSI( SPI_GPIO_PORT, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

    SSIConfigSetExpClk( SPI_PORT, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 100000, 8);

    while( SSIDataGetNonBlocking( SPI_PORT, &rx)) {
    	;
    }

    SSIIntRegister( SPI_PORT, isrSSI1);
//    SSIIntEnable( SSI0_BASE, /* SSI_TXFF | */ SSI_RXFF | SSI_RXTO /*| SSI_RXOR */);

	IntPrioritySet( INT_SSI1, IRQPRIO_SSI);

    SSIIntEnable( SPI_PORT, SSI_RXTO);

    SSIEnable(SPI_PORT);

	//-----------------------------------------------------------------
    // QEI decoder

	ENABLE_PERIPHERAL( QEI_PERI);
	ENABLE_PERIPHERAL( QEI_GPIO_PERI);

	HWREG( QEI_GPIO_PORT + GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG( QEI_GPIO_PORT + GPIO_O_CR) |= QEI_MASK;

    GPIOPinConfigure(GPIO_PD6_PHA0);
    GPIOPinConfigure(GPIO_PD7_PHB0);    // now redundant

    // Tell the GPIO module which pins will be QEI type pins
    GPIOPinTypeQEI( QEI_GPIO_PORT, GPIO_PIN_7 | GPIO_PIN_6);
    GPIOPadConfigSet( QEI_GPIO_PORT, GPIO_PIN_7 | GPIO_PIN_6, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    QEIConfigure( QEI_PORT, (QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), 24);

    QEIEnable( QEI_PORT);

    // configure velocity settings
    // 18000rpm = 300rps * 24ppr = 7200pps @ 0.25s = 1800
    // rpm/10 @ 4Hz
    QEIVelocityConfigure( QEI_PORT, QEI_VELDIV_1, 20000000);
    QEIVelocityEnable( QEI_PORT);

    QEIIntRegister( QEI_PORT, isrQEI0);

    IntPrioritySet( INT_QEI0, IRQPRIO_QEI);
    QEIIntEnable( QEI_PORT, QEI_INTTIMER);

//    QEIDisable( QEI_BASE);
	HWREG( QEI_GPIO_PORT + GPIO_O_LOCK) = 0;
}

uint32_t getVelocity() {
	return QEIVelocityGet( QEI_GPIO_PORT);
}

uint32_t setVelocity( uint32_t rpm) {
	rpmTargetValue = rpm;

	return rpm;
}

uint32_t setPWM( uint32_t pwm) {
	if ( pwm > PWM_MAX) pwm = PWM_MAX;

	//TimerPrescaleMatchSet( ESC_WTIMER_BASE, TIMER_A, PWM_PRESCALE);
	TimerMatchSet( ESC_WTIMER_BASE, TIMER_A, pwm + PWM_BASE);

	return pwm;
}

#endif
