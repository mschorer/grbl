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
#include "io_mcp23008.h"

// i2c transaction queue and current op

t_i2cOpQueue opQueue;
volatile t_i2cStatus currOp;

volatile t_i2cDataBuffer i2cData;

// set up function

void TWI_init(void) {

	// init buffers
	opQueue.head = opQueue.tail = 0;
	currOp.packet = 0;
	currOp.didx = 0;

	i2cData.tail = i2cData.head = 0;

    //enable I2C module 0
    SysCtlPeripheralEnable( TIVA_I2C_PERI);
    SysCtlPeripheralEnable( SYSCTL_PERIPH_TIMER1);
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
    // use direct register acccess to avoid secondary effexts of GpioPadConfigSet
    GPIO_PORTB_AHB_PUR_R |= TIVA_I2C_PINS;

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

    TimerConfigure( TIMER1_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet( TIMER1_BASE, TIMER_A, TIMER_GET_DELAY_HZ( 1000));
	TimerEnable( TIMER1_BASE, TIMER_A);

    TimerIntRegister( TIMER1_BASE, TIMER_TIMA_TIMEOUT, TWI_isrTick);
	IntPrioritySet( INT_TIMER1A, IRQPRIO_LED);
	TimerIntClear( TIMER1_BASE, TimerIntStatus( TIMER1_BASE, true));
    TimerIntEnable( TIMER1_BASE, TIMER_TIMA_TIMEOUT);
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

// construct an i2c-transaction
void TWI_putOp( uint8_t addr, uint8_t dirBit, uint8_t* data, uint32_t len, void (*complete)( uint8_t* data, uint32_t len)) {
	t_i2cTransfer i2t;

	i2t.cmd = (addr << 1) | (dirBit & I2C_DIR_MASK);
	i2t.data = TWI_malloc( data, len);
	i2t.len = len;
	i2t.complete = complete;

	TWI_putQueue( &i2t);
}

uint8_t* TWI_malloc( uint8_t *buffer, uint32_t len) {
	if ( len <= 0) return 0;

	if ( i2cData.head + len > I2C_DATA_SIZE) i2cData.head = 0;

	uint8_t* rc = (uint8_t*) memcpy( (void*)&( i2cData.data[ i2cData.head]), (void*) buffer, len);

	i2cData.head += len;

	return rc;
}

bool TWI_free( uint8_t *buffer, uint32_t len) {
	return true;
}

void TWI_isrTick() {
	TWI_triggerSend();
}

bool TWI_isBusy() {
	return (( opQueue.head != opQueue.tail) && currOp.packet != 0);
}

void isrI2Cmaster() {
	bool restartTransfer = false;

	bool iState = I2CMasterIntStatus( TIVA_I2C_BASE, true);
	uint32_t ixState = I2CMasterIntStatusEx( TIVA_I2C_BASE, true);

	I2CMasterIntClear( TIVA_I2C_BASE);
	I2CMasterIntClearEx( TIVA_I2C_BASE, ixState);

	uint32_t i2cErr = I2CMasterErr( TIVA_I2C_BASE);

	if ( i2cErr != I2C_MASTER_ERR_NONE) {
		// if an error occurs: cancel current transaction, start next

		if ( i2cErr & ( I2C_MASTER_ERR_ADDR_ACK | I2C_MASTER_ERR_DATA_ACK)) {
			currOp.packet = 0;
		}

		if ( i2cErr & ( I2C_MASTER_ERR_ARB_LOST | I2C_MASTER_ERR_CLK_TOUT)) {
			currOp.packet = 0;
		}

		restartTransfer = true;
	}

	if ( ixState & I2C_MASTER_INT_START) {
		// placeholder
		restartTransfer = false;
	}

	if ( ixState & ( I2C_MASTER_INT_STOP | I2C_MASTER_INT_TIMEOUT)) {
		// when a transaction is completed, trigger next one
		currOp.packet = 0;
		// start next transfer
		restartTransfer = true;
	}

	if ( i2cErr == I2C_MASTER_ERR_NONE && ixState & I2C_MASTER_INT_DATA) {
        // handle data

		if (( currOp.packet->cmd & I2C_DIR_MASK) == I2C_READ) {

			// a byte was received, move it to the buffers
			currOp.packet->data[ currOp.didx++] = I2CMasterDataGet(TIVA_I2C_BASE);

			// are we done?
			if ( currOp.didx == currOp.packet->len) {
				// finish send
				if ( currOp.packet->len > 1) I2CMasterControl(TIVA_I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);

				// is there a "complete" handler? call it
				if ( currOp.packet->complete) currOp.packet->complete( currOp.packet->data, currOp.packet->len);
				currOp.packet = 0;
				restartTransfer = true;
			} else {
				// prep for sending next byte
				I2CMasterControl(TIVA_I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
			}
		} else {
			// put next byte to the xmit buffer
			uint8_t data = currOp.packet->data[ currOp.didx++];

			if ( currOp.didx <= currOp.packet->len ) I2CMasterDataPut(TIVA_I2C_BASE, data);

			if ( currOp.didx < currOp.packet->len) {
				// send and continue
				I2CMasterControl(TIVA_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
			} else {
				// send byte and finish transfer
				if ( currOp.packet->len > 1) I2CMasterControl(TIVA_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);

				// is there a complete handler? call it
				if ( currOp.packet->complete) currOp.packet->complete( currOp.packet->data, currOp.packet->len);
				currOp.packet = 0;
				restartTransfer = true;
			}
		}
	}

	if ( restartTransfer) {
//		TWI_triggerSend();
	}
}

// start the next i2c transaction
bool TWI_triggerSend() {

	if ( opQueue.head == opQueue.tail || currOp.packet != 0) return false;

	if ( I2CMasterBusy( TIVA_I2C_BASE)) {
		return false;
	}

	currOp.didx = 0;
	currOp.packet = &opQueue.ops[ opQueue.tail];
	opQueue.tail = ++opQueue.tail % I2C_OPQ_SIZE;

	uint8_t addr = currOp.packet->cmd >> 1;
/*
	if ( addr == MCP_ADDR) {
		uint8_t temp = 0;
	}
*/
	if (( currOp.packet->cmd & I2C_DIR_MASK) == I2C_READ) {
	    // set slave address, do read
	    I2CMasterSlaveAddrSet(TIVA_I2C_BASE, addr, true);

	    // trigger read, the interrupt will bring in the data
		I2CMasterControl(TIVA_I2C_BASE, ( currOp.packet->len == 1) ? I2C_MASTER_CMD_SINGLE_RECEIVE : I2C_MASTER_CMD_BURST_RECEIVE_START);
	} else {
	    // set slave address, do write
	    I2CMasterSlaveAddrSet(TIVA_I2C_BASE, addr, false);

	    // trigger send, the first byte of data will be passed in via interrupt
	    I2CMasterDataPut(TIVA_I2C_BASE, currOp.packet->data[ currOp.didx++]);
		I2CMasterControl(TIVA_I2C_BASE, ( currOp.packet->len == 1) ? I2C_MASTER_CMD_SINGLE_SEND : I2C_MASTER_CMD_BURST_SEND_START);
	}

	return true;
}

#endif
