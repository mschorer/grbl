/*
 * i2c_tiva.c
 *
 *  Created on: 20.05.2015
 *      Author: legscmh
 */

#include "cpu_map.h"

#ifdef CPU_MAP_TIVA

#include <inc/tm4c123gh6pm.h>

#include <inc/hw_memmap.h>
#include <inc/hw_gpio.h>
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <inc/hw_i2c.h>
#include <inc/hw_timer.h>
#include <inc/hw_sysctl.h>

#include <driverlib/pin_map.h>

#include <driverlib/gpio.h>
#include <driverlib/i2c.h>
#include <driverlib/interrupt.h>
#include <driverlib/sysctl.h>
#include <driverlib/timer.h>

#include "i2c_tiva.h"

//initialize I2C module 0

#define I2C_Q_SIZE	240

volatile uint8_t i2c_q_head = 0;
volatile uint8_t i2c_q_tail = 0;
uint8_t i2c_q[ I2C_Q_SIZE];

#define I2C_BASE	I2C3_BASE

// simulated registers for comm with imx6
//t_regStack regStack[NUM_SSI_DATA];

//Slightly modified version of TI's example code

void TWI_init(void) {
    //enable I2C module 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C3);
    //enable GPIO peripheral that contains I2C 3
	SysCtlGPIOAHBEnable( SYSCTL_PERIPH_GPIOD); \
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlDelay( 3);

    //reset module
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C3);

    // Select the I2C function for these pins.
    GPIOPinTypeI2CSCL(GPIO_PORTD_AHB_BASE, GPIO_PIN_0);
    GPIOPinTypeI2C(GPIO_PORTD_AHB_BASE, GPIO_PIN_1);

//    GPIOPadConfigSet( GPIO_PORTD_AHB_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    // Configure the pin muxing for I2C0 functions on port B2 and B3.
    GPIOPinConfigure(GPIO_PD0_I2C3SCL);
    GPIOPinConfigure(GPIO_PD1_I2C3SDA);

    // enable on-chip pullups, just to be sure
    GPIO_PORTD_AHB_PUR_R |= ( GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

    // Enable and initialize the I2C0 master module.  Use the system clock for
    // the I2C0 module.  The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.
    I2CMasterInitExpClk(I2C_BASE, SysCtlClockGet(), false);

    I2CIntRegister( I2C_BASE, isrI2Cmaster);

    /* Enable interrupts */
    I2CMasterIntClear( I2C_BASE);
    I2CMasterIntEnable( I2C_BASE);
    I2CMasterIntEnableEx( I2C_BASE, I2C_MASTER_INT_MASK);

    //clear I2C FIFOs
//    HWREG(I2C_BASE + I2C_O_FIFOCTL) = 80008000;

    IntEnable( INT_I2C3);
}

bool TWI_tick() {
	return true;
}

bool TWI_queue( uint8_t *buf, uint32_t len) {
	if ( TWI_isBusy()) return false;

	uint32_t i = 0;
	while( len--) {
		i2c_q[ i2c_q_head++] = buf[i];
		if ( i2c_q_head >= I2C_Q_SIZE) i2c_q_head = 0;
		i++;
	}

	return true;
}

bool TWI_isBusy() {
	return ( i2c_q_head != i2c_q_tail);
}

void TWI_prep_data() {
    I2CMasterDataPut(I2C_BASE, i2c_q[ i2c_q_tail++]);
    if ( i2c_q_tail >= I2C_Q_SIZE) i2c_q_tail = 0;
}

void isrI2Cmaster() {
	static volatile uint8_t i2cRegIdx = 0;
	static volatile uint8_t i2cByteIdx = 0;
	static volatile uint8_t i2cDEV = 0;
	static volatile uint8_t i2cRW = 0;
	static volatile uint8_t i2cData = 0;
	static volatile uint32_t i2cNOCs = 0;
	// uint32_t i2cStatus;

	bool iState = I2CMasterIntStatus( I2C_BASE, true);
	I2CMasterIntClear( I2C_BASE);

	uint32_t i2cErr = I2CMasterErr( I2C_BASE);

	uint32_t ixState = I2CMasterIntStatusEx( I2C_BASE, true);
	I2CMasterIntClearEx( I2C_BASE, ixState);

	if ( i2cErr != I2C_MASTER_ERR_NONE) {
		if ( i2cErr & ( I2C_MASTER_ERR_ADDR_ACK | I2C_MASTER_ERR_DATA_ACK)) {
			i2c_q_tail = i2c_q_head = 0;
		}

		if ( i2cErr & ( I2C_MASTER_ERR_ARB_LOST | I2C_MASTER_ERR_CLK_TOUT)) {
			i2c_q_tail = i2c_q_head = 0;
		}
	}

	if ( ixState & I2C_MASTER_INT_TIMEOUT) {
		i2c_q_tail = i2c_q_head = 0;
	}

	if ( ixState & I2C_MASTER_INT_DATA) {
        //put last piece of data into I2C FIFO
		TWI_prep_data();

        switch( i2c_q_tail - i2c_q_head) {
        case 0:
            //send next data that was just placed into FIFO
            I2CMasterControl(I2C_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
        	break;

        default:
            //send next data that was just placed into FIFO
            I2CMasterControl(I2C_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
        }
	}

}

//sends an array of data via I2C to the specified slave
void TWI_send(uint8_t slave_addr)
{
    // set slave address
    I2CMasterSlaveAddrSet(I2C_BASE, slave_addr, false);

    // send first byte of payload
	TWI_prep_data();

    //if there is only one argument, we only need to use the
    //single send I2C function
    if( i2c_q_head == i2c_q_tail) {
        //Initiate send of data from the MCU
        I2CMasterControl(I2C_BASE, I2C_MASTER_CMD_SINGLE_SEND);

    } else {
        //Initiate send of data from the MCU
        I2CMasterControl(I2C_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    }
}


/*
void InitI2Cslave() {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    SysCtlDelay( 3);

    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    GPIOPinConfigure(GPIO_PCTL_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PCTL_PB3_I2C0SDA);

    I2CSlaveEnable( I2C_BASE);
    I2CSlaveAddressSet( I2C_BASE, 0, SLAVE_OWN_ADDRESS);

    // as the  schematic doesn't have dedicated pullups, we need to enable the chip's internal ones
    GPIO_PORTB_PUR_R |= ( GPIO_PIN_3 | GPIO_PIN_2);

    // we let the hardware control ACK automatically
    I2CSlaveACKOverride( I2C_BASE, false);

//    I2CSlaveFIFOEnable( I2C_BASE, I2C_SLAVE_TX_FIFO_ENABLE);
//    I2CTxFIFOConfigSet( I2C_BASE, I2C_FIFO_CFG_TX_SLAVE | I2C_FIFO_CFG_TX_TRIG_1);

    I2CIntRegister( I2C_BASE, isrI2C0slave);

    // Enable interrupts
    I2CSlaveIntEnableEx( I2C_BASE, I2C_SLAVE_INT_MASK);
    I2CSlaveIntEnable( I2C_BASE);

    IntEnable(INT_I2C0);
}

void isrI2C0slave() {
	static volatile uint8_t i2cRegIdx = 0;
	static volatile uint8_t i2cByteIdx = 0;
	static volatile uint8_t i2cDEV = 0;
	static volatile uint8_t i2cRW = 0;
	static volatile uint8_t i2cData = 0;
	static volatile uint32_t i2cNOCs = 0;
	uint32_t i2cStatus;

	bool iState = I2CSlaveIntStatus( I2C_BASE, true);
	I2CSlaveIntClear( I2C_BASE);

	uint32_t ixState = I2CSlaveIntStatusEx( I2C_BASE, true);
	I2CSlaveIntClearEx( I2C_BASE, ixState);

	if ( ixState & I2C_SLAVE_INT_START) {
		I2CSlaveACKValueSet( I2C_BASE, true);
	}

	i2cStatus = I2CSlaveStatus( I2C_BASE);

	switch( i2cStatus) {

		case I2C_SLAVE_ACT_RREQ_FBR:
			// here when Master sends exact 1 B - normal write
			i2cRegIdx = I2CSlaveDataGet(I2C_BASE);
			i2cByteIdx = 0;
			break;

		case I2C_SLAVE_ACT_RREQ:
			// here when Master sends WRITE and more than 1 B to send - burst write
			i2cData = I2CSlaveDataGet(I2C_BASE);

			regStack[ i2cRegIdx].b[i2cByteIdx++] = i2cData;

			if ( i2cByteIdx >= 4) {

				// update hw only if full 32-bit value is received
				switch( i2cRegIdx) {


				default:
					;
				}

				i2cByteIdx = 0;
				i2cRegIdx++;
			}
			break;

		case I2C_SLAVE_ACT_TREQ:
			I2CSlaveDataPut(I2C_BASE, regStack[ i2cRegIdx].b[ i2cByteIdx++]);
/ *
 * i2c fifos are not available on 4c123
			I2CFIFODataPutNonBlocking(I2C_BASE, ssiRegisters[ i2cRegIdx].b[ i2cByteIdx++]);
			I2CFIFODataPutNonBlocking(I2C_BASE, ssiRegisters[ i2cRegIdx].b[ i2cByteIdx++]);
			I2CFIFODataPutNonBlocking(I2C_BASE, ssiRegisters[ i2cRegIdx].b[ i2cByteIdx++]);
			I2CFIFODataPutNonBlocking(I2C_BASE, ssiRegisters[ i2cRegIdx].b[ i2cByteIdx++]);
* /
			if ( i2cByteIdx >= 4) {
				i2cByteIdx = 0;
				i2cRegIdx++;
			}
			break;

		case I2C_SLAVE_ACT_QCMD:
//			I2CSlaveDataPut(I2C_BASE, 0xa6);
			i2cDEV = I2CSlaveDataGet(I2C_BASE);
			i2cRW = i2cDEV & 0x01;
			i2cDEV >>= 1;
			break;

		case I2C_SLAVE_ACT_OWN2SEL:
		case I2C_SLAVE_ACT_QCMD_DATA:
		default:
		    i2cNOCs++;
	}
}
*/
/*
//sends an I2C command to the specified slave
void I2CSend(uint8_t slave_addr, uint8_t num_of_args, ...) {
	uint8_t i;

    // Tell the master module what address it will place on the bus when
    // communicating with the slave.
    I2CMasterSlaveAddrSet(I2C_BASE, slave_addr, false);

    //stores list of variable number of arguments
    va_list vargs;

    //specifies the va_list to "open" and the last fixed argument
    //so vargs knows where to start looking
    va_start(vargs, num_of_args);

    //put data to be sent into FIFO
    I2CMasterDataPut(I2C_BASE, va_arg(vargs, uint32_t));

    //if there is only one argument, we only need to use the
    //single send I2C function
    if(num_of_args == 1)
    {
        //Initiate send of data from the MCU
        I2CMasterControl(I2C_BASE, I2C_MASTER_CMD_SINGLE_SEND);

        // Wait until MCU is done transferring.
        while(I2CMasterBusy(I2C_BASE));

        //"close" variable argument list
        va_end(vargs);
    }

    //otherwise, we start transmission of multiple bytes on the
    //I2C bus
    else
    {
        //Initiate send of data from the MCU
        I2CMasterControl(I2C_BASE, I2C_MASTER_CMD_BURST_SEND_START);

        // Wait until MCU is done transferring.
        while(I2CMasterBusy(I2C_BASE));

        //send num_of_args-2 pieces of data, using the
        //BURST_SEND_CONT command of the I2C module
        for(i = 1; i < (num_of_args - 1); i++)
        {
            //put next piece of data into I2C FIFO
            I2CMasterDataPut(I2C_BASE, va_arg(vargs, uint32_t));
            //send next data that was just placed into FIFO
            I2CMasterControl(I2C_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);

            // Wait until MCU is done transferring.
            while(I2CMasterBusy(I2C_BASE));
        }

        //put last piece of data into I2C FIFO
        I2CMasterDataPut(I2C_BASE, va_arg(vargs, uint32_t));
        //send next data that was just placed into FIFO
        I2CMasterControl(I2C_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
        // Wait until MCU is done transferring.
        while(I2CMasterBusy(I2C_BASE));

        //"close" variable args list
        va_end(vargs);
    }
}

*/
#endif
