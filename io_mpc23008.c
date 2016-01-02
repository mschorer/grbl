/*
 * io_mpc23008.c
 *
 *  Created on: 14.12.2015
 *      Author: legscmh
 */

#include "system.h"
#include "motion_control.h"
#include "cpu_map.h"

#include "i2c_master.h"
#include "i2c_tiva.h"

#include "machine_control.h"
#include "io_mcp23008.h"

// read port

uint8_t mcp_setup[12]	= { 0x00, MCP_MASK_INPUTS, 0x00, MCP_MASK_INPUTS, MCP_MASK_INPUTS, MCP_MASK_INPUTS, 0x00, MCP_MASK_INPUTS, 0x00, 0x00, MCP_MASK_OUTPUTS};
uint8_t mcp_send[16];
uint8_t mcp_read[16];
uint8_t mcp_cmd;

/*
 * default register values
 *
 * MCP_IODIR		0x3f	// 0, bits 7/6 out, 5-0 in
 * MCP_IPOL			0x00	// 1, normal polarity
 * MCP_GPINTEN		0x3f	// 2, enable int-on-change
 * MCP_DEFVAL		0x3f	// 3, int def val
 * MCP_INTCON		0x3f	// 4, compare against def val
 * MCP_IO_CON		0x00	// 5, seqop on, slew on, opendrain off, int active low
 * MCP_GPPU			0x3f	// 6, enable internal pullup on inputs
 * MCP_INTF			---		// 7, read only
 * MCP_INTCAP		---		// 8, ro, io captured when int triggered
 * MCP_GPIO			0xc0	// 9, set outputs to high/inactive
 * MCP_OLAT			0xc0	// a, output latches
 */

GLOBAL_INT_VECTOR( io_isrFault);

void io_init() {
	ENABLE_PERIPHERAL( I2CINT_PERI);

	GPIO_INPUT_SET( I2CINT_DDR, I2CINT_MASK);

	GPIOIntDisable( I2CINT_PORT, I2CINT_MASK);
	GPIOIntTypeSet( I2CINT_PORT, I2CINT_MASK, GPIO_BOTH_EDGES);

    GPIOPadConfigSet( I2CINT_PORT, I2CINT_MASK, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

	GPIOIntRegister( I2CINT_PORT, io_isrFault);
	GPIOIntClear( I2CINT_PORT, I2CINT_MASK);

	IntPrioritySet( INT_GPIOC, IRQPRIO_FAULT);

	GPIOIntEnable( I2CINT_PORT, I2CINT_MASK);

	io_setIndex( 0);
	io_sendCmd( MCP_ADDR, true, mcp_setup, 11, 0);
}

void io_sendCmd( uint8_t addr, bool doWrite, uint8_t* data, uint32_t len, void (*complete)( uint8_t* data, uint32_t len)) {
	t_i2cTransfer i2t;

	i2t.cmd = addr << 1;
	i2t.cmd |= doWrite ? I2C_WRITE : I2C_READ;
	i2t.data = data;
	i2t.len = len;
	i2t.complete = complete;

	TWI_putQueue( &i2t);
}

void io_setIndex( uint8_t reg) {
	mcp_cmd = reg;
	io_sendCmd( MCP_ADDR, true, &mcp_cmd, 1, 0);
}

void io_setSleep( bool sleep) {
	// set addr to GPIO, as autoinc is enabled the second write goes to OLAT (which is exactly what we want)
	// "pull reset", write to GPIO
	// "deactivate reset", write to OLAT
	mcp_send[0] = MCP_REG_GPIO;
	mcp_send[1]	= (( 1<<MCP_PIN_RESET) | ( sleep ? 0 : ( 1<<MCP_PIN_SLEEP)));

	io_sendCmd( MCP_ADDR, true, mcp_send, 2, 0);
}

void io_resetFault() {
	// set addr to GPIO, as autoinc is enabled the second write goes to OLAT (which is exactly what we want)
	// "pull reset", write to GPIO
	// "deactivate reset", write to OLAT
	mcp_send[0] = MCP_REG_GPIO;
	mcp_send[1]	= (( 0<<MCP_PIN_RESET) | ( 1<<MCP_PIN_SLEEP));
	mcp_send[2] = (( 1<<MCP_PIN_RESET) | ( 1<<MCP_PIN_SLEEP));

	io_sendCmd( MCP_ADDR, true, mcp_send, 3, 0);
}

void io_triggerReadInputs() {
	io_setIndex( MCP_REG_GPIO);
	io_sendCmd( MCP_ADDR, false, mcp_read, 1, io_read_complete);
}

// this is called after the FAULT bits have been read from the io expander
void io_read_complete( uint8_t* data, uint32_t len) {
	char msg[16];
	char axes[6] = "XYZABC";
	uint8_t i, mask = 0x20;

	if ( data == mcp_read && len == 1) {
		uint8_t error_bits = mcp_read[0] & MCP_MASK_INPUTS;

		if ( error_bits != MCP_MASK_INPUTS) {
			// raise alarm
			if (bit_isfalse(sys.execute,EXEC_ALARM)) {
				mc_reset(); // Initiate system kill.
				bit_true_atomic(sys.execute, (EXEC_ALARM | EXEC_CRIT_EVENT)); // Indicate critical event
			}

			// reset drivers
			io_resetFault();

			strcpy( msg, "FAULT ");
			for( i = 0; i < 6; i++) {
				msg[ 6+i] = (data[0] & mask) ? '-' : axes[i];
				mask >>= 1;
			}
			msg[ 6+i] = '\0';

			mctrl_queueMsgString( msg);
		}
	}
}

// interrupt handler for the "FAULT" interrupt
void io_isrFault() {
	uint32_t status = GPIOIntStatus( I2CINT_PORT, true);
	GPIOIntClear( I2CINT_PORT, status);

	// set GPIO addr, read back data
	// can be called in interrupt context as data/cmd is just posted to the buffers
	// the "complete handler" gets called when data has arrived
	io_triggerReadInputs();

/*
 * probably this is the same:
 * send address
 * send read command
 * 	mcp_send[0] = MCP_REG_GPIO;
	mcp_send[1]	= ( MCP_ADDR << 1) | I2C_READ;
	io_sendCmd( MCP_ADDR, false, mcp_send, 2, io_mcp_read_complete);
 *
 */
}

