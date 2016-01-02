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

// i2c transaction queue and current op

t_i2cOpQueue opQueue;
volatile t_i2cStatus currOp;

// set up function

void TWI_init(void) {

	// init buffers
	opQueue.head = opQueue.tail = 0;
	currOp.packet = 0;
	currOp.didx = 0;

    //enable I2C module 0
    SysCtlPeripheralEnable( TIVA_I2C_PERI);
    //enable GPIO peripheral that contains I2C0
    SysCtlPeripheralEnable( TIVA_I2C_PINPERI);
    SysCtlDelay( 3);

    //reset module
    SysCtlPeripheralReset( TIVA_I2C_PERI);

    // Select the I2C function for these pins.
    GPIOPinTypeI2CSCL( TIVA_I2C_PINPORT, GPIO_PIN_2);
    GPIOPinTypeI2C( TIVA_I2C_PINPORT, GPIO_PIN_3);

    // Configure the pin muxing for I2C0 functions on port B2 and B3.
    GPIOPinConfigure( TIVA_I2C_SCL);
    GPIOPinConfigure( TIVA_I2C_SDA);

    // enable on-chip pullups, just to be sure
    GPIOPadConfigSet( TIVA_I2C_PINPORT, TIVA_I2C_PINS, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    //GPIO_PORTB_AHB_PUR_R |= TIVA_I2C_PINS;	//| GPIO_PIN_2 | GPIO_PIN_3);

    // Enable and initialize the I2C0 master module.  Use the system clock for
    // the I2C0 module.  The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.
    I2CMasterInitExpClk( TIVA_I2C_BASE, SysCtlClockGet(), false);

    I2CIntRegister( TIVA_I2C_BASE, isrI2Cmaster);

    /* Enable interrupts */
    I2CMasterIntClear( TIVA_I2C_BASE);
    I2CMasterIntEnable( TIVA_I2C_BASE);
    I2CMasterIntEnableEx( TIVA_I2C_BASE, I2C_MASTER_INT_MASK);

    //clear I2C FIFOs
//    HWREG(I2C_BASE + I2C_O_FIFOCTL) = 80008000;

	IntPrioritySet( INT_I2C0, IRQPRIO_I2C);

    IntEnable( INT_I2C0);
}

// copy a transaction into the buffer
// if op == null, a transaction is "reserved" that can be filled afterwards
//TODO: fix race between filling in the data and an interrupt finishing the last transaction and starting a new
t_i2cTransfer* TWI_putQueue( t_i2cTransfer* op) {
	t_i2cTransfer* rc = &opQueue.ops[ opQueue.head];

	if ( op != 0) memcpy( rc, op, sizeof( t_i2cTransfer));

	opQueue.head = ++opQueue.head % I2C_OPQ_SIZE;

	if ( opQueue.head == opQueue.tail) {
		opQueue.tail = ++opQueue.tail % I2C_OPQ_SIZE;
	}

	return rc;
}

// get the next transaction off the buffer
//TODO: fix in-place operation
t_i2cTransfer* TWI_fetchQueue() {
	t_i2cTransfer* rc = 0;

	if ( opQueue.head != opQueue.tail) {
		rc = &opQueue.ops[ opQueue.tail];
		opQueue.tail = ++opQueue.tail % I2C_OPQ_SIZE;
	}

	return rc;
}

// send a packet
// used for tool/spindle control
bool TWI_post( uint8_t slave_addr, uint8_t *buf, uint32_t len) {
	if ( len == 0) return false;

	t_i2cTransfer* t = TWI_putQueue( 0);

	t->cmd = ( slave_addr << 1) | I2C_WRITE;
	t->data = buf;
	t->len = len;
	t->complete = 0;

	return true;
}

bool TWI_tick() {
	return true;
}

bool TWI_isBusy() {
	return (( opQueue.head != opQueue.tail) && currOp.packet != 0);
}

void isrI2Cmaster() {

	bool iState = I2CMasterIntStatus( TIVA_I2C_BASE, true);
	I2CMasterIntClear( TIVA_I2C_BASE);

	uint32_t i2cErr = I2CMasterErr( TIVA_I2C_BASE);

	uint32_t ixState = I2CMasterIntStatusEx( TIVA_I2C_BASE, true);
	I2CMasterIntClearEx( TIVA_I2C_BASE, ixState);

	if ( i2cErr != I2C_MASTER_ERR_NONE) {
		// if an error occurs: cancel current transaction, start next

		if ( i2cErr & ( I2C_MASTER_ERR_ADDR_ACK | I2C_MASTER_ERR_DATA_ACK)) {
			currOp.packet = 0;
		}

		if ( i2cErr & ( I2C_MASTER_ERR_ARB_LOST | I2C_MASTER_ERR_CLK_TOUT)) {
			currOp.packet = 0;
		}

		TWI_triggerSend();
	}

	if ( ixState & I2C_MASTER_INT_START) {
		// placeholder
	}

	if ( ixState & ( I2C_MASTER_INT_STOP | I2C_MASTER_INT_TIMEOUT)) {
		// when a transaction is completed, trigger next one
		currOp.packet = 0;
		// start next transfer
		TWI_triggerSend();
	}

	if ( ixState & I2C_MASTER_INT_DATA) {
        // handle data

		if (( currOp.packet->cmd & I2C_DIR_MASK) == I2C_READ) {

			// a byte was received, move it to the buffers
			currOp.packet->data[ currOp.didx++] = I2CMasterDataGet(TIVA_I2C_BASE);

			// are we done?
			if ( currOp.didx == currOp.packet->len) {
	            // finish send
	            I2CMasterControl(TIVA_I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);

	            // is there a "complete" handler? call it
	            if ( currOp.packet->complete) currOp.packet->complete( currOp.packet->data, currOp.packet->len);
	            currOp.packet = 0;
	        } else {
	            // prep for sending next byte
	            I2CMasterControl(TIVA_I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
	        }
		} else {
			// are we done?
			if ( currOp.didx == currOp.packet->len) {
	            // send byte and finish transfer
	            I2CMasterControl(TIVA_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);

	            // is there a complete handler? call it
	            if ( currOp.packet->complete) currOp.packet->complete( currOp.packet->data, currOp.packet->len);
	            currOp.packet = 0;
	        } else {
				// put next byte to the xmit buffer
				I2CMasterDataPut(TIVA_I2C_BASE, currOp.packet->data[ currOp.didx++]);

				// send and continue
	            I2CMasterControl(TIVA_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
	        }
		}
	}

}

// start the next i2c transaction
bool TWI_triggerSend() {

	if ( opQueue.head == opQueue.tail || currOp.packet != 0) return false;

	currOp.packet = &opQueue.ops[ opQueue.tail];
	opQueue.tail = ++opQueue.tail % I2C_OPQ_SIZE;

	if (( currOp.packet->cmd & I2C_DIR_MASK) == I2C_READ) {
	    // set slave address, do read
	    I2CMasterSlaveAddrSet(TIVA_I2C_BASE, currOp.packet->cmd >> 1, true);

	    // trigger read, the interrupt will bring in the data
		I2CMasterControl(TIVA_I2C_BASE, ( currOp.packet->len == 0) ? I2C_MASTER_CMD_SINGLE_RECEIVE : I2C_MASTER_CMD_BURST_RECEIVE_START);
	} else {
	    // set slave address, do write
	    I2CMasterSlaveAddrSet(TIVA_I2C_BASE, currOp.packet->cmd >> 1, false);

	    // put data, trigger send
		I2CMasterDataPut(TIVA_I2C_BASE, currOp.packet->data[ currOp.didx++]);
		I2CMasterControl(TIVA_I2C_BASE, ( currOp.packet->len == 0) ? I2C_MASTER_CMD_SINGLE_SEND : I2C_MASTER_CMD_BURST_SEND_START);
	}

	return true;
}

#endif
