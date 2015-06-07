/*
 * i2c_tiva.h
 *
 *  Created on: 20.05.2015
 *      Author: legscmh
 */

#ifndef I2C_TIVA_H_
#define I2C_TIVA_H_

//---------------------------------------------------------------------------------

// I2C

#define SLAVE_OWN_ADDRESS	0x5C

// I2C int sources
//#define I2C_INT_MASK ( I2C_SLAVE_INT_START | I2C_SLAVE_INT_STOP | I2C_SLAVE_INT_DATA | I2C_SLAVE_INT_RX_FIFO_FULL | I2C_SLAVE_INT_TX_FIFO_EMPTY | I2C_SLAVE_INT_RX_FIFO_REQ | I2C_SLAVE_INT_TX_FIFO_REQ | I2C_SLAVE_INT_TX_DMA_DONE | I2C_SLAVE_INT_RX_DMA_DONE)
#define I2C_SLAVE_INT_MASK ( I2C_SLAVE_INT_DATA)	// | I2C_SLAVE_INT_START | I2C_SLAVE_INT_STOP )

/*
	I2C_MASTER_INT_RX_FIFO_FULL - RX FIFO Full interrupt
	I2C_MASTER_INT_TX_FIFO_EMPTY - TX FIFO Empty interrupt
	I2C_MASTER_INT_RX_FIFO_REQ - RX FIFO Request interrupt
	I2C_MASTER_INT_TX_FIFO_REQ - TX FIFO Request interrupt
	I2C_MASTER_INT_ARB_LOST - Arbitration Lost interrupt
	I2C_MASTER_INT_STOP - Stop Condition interrupt
	I2C_MASTER_INT_START - Start Condition interrupt
	I2C_MASTER_INT_NACK - Address/Data NACK interrupt
	I2C_MASTER_INT_TX_DMA_DONE - TX DMA Complete interrupt
	I2C_MASTER_INT_RX_DMA_DONE - RX DMA Complete interrupt
	I2C_MASTER_INT_TIMEOUT - Clock Timeout interrupt
	I2C_MASTER_INT_DATA - Data interrupt
*/

#define I2C_MASTER_INT_MASK 0xfff
//( I2C_MASTER_INT_DATA | I2C_MASTER_INT_TIMEOUT | I2C_MASTER_INT_RX_FIFO_FULL | I2C_MASTER_INT_TX_FIFO_EMPTY | I2C_MASTER_INT_RX_FIFO_REQ | I2C_MASTER_INT_TX_FIFO_REQ | I2C_MASTER_INT_ARB_LOST | I2C_MASTER_INT_STOP | I2C_MASTER_INT_START | I2C_MASTER_INT_NACK)

// internal "register" space
#define NUM_SSI_DATA 32

typedef union {
	uint8_t b[4];
	uint16_t h[2];
	uint32_t u;
	float f;
} t_regStack;

//---------------------------------------------------------------------------------

void TWI_init();
bool TWI_tick();

bool TWI_queue(uint8_t *buffer, uint8_t write_bytes);
bool TWI_isBusy();
void TWI_send( uint8_t adr);

void isrI2C0master();
void isrI2C0slave();

#endif /* I2C_TIVA_H_ */
