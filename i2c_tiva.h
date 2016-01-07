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

#define I2C_DIR_MASK		0x01
#define I2C_WRITE			0x00
#define I2C_READ			0x01

// number of pending i2c transfers
#define I2C_OPQ_SIZE		16

#define I2C_DATA_SIZE		256

// I2C int sources
#define I2C_MASTER_INT_MASK ( I2C_MASTER_INT_START | I2C_MASTER_INT_STOP | I2C_MASTER_INT_DATA | I2C_MASTER_INT_TIMEOUT | I2C_MASTER_INT_NACK)
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

/*
// internal "register" space
#define NUM_SSI_DATA 32

typedef union {
	uint8_t b[4];
	uint16_t h[2];
	uint32_t u;
	float f;
} t_regStack;
*/

typedef struct {
	uint8_t cmd;
	uint8_t* data;
	uint32_t len;
	void (*complete)( uint8_t* data, uint32_t len);
} t_i2cTransfer;

typedef struct {
	t_i2cTransfer* packet;
	int32_t didx;
} t_i2cStatus;

typedef struct {
	uint32_t head;
	uint32_t tail;
	t_i2cTransfer ops[ I2C_OPQ_SIZE];
} t_i2cOpQueue;

typedef struct {
	uint32_t head;
	uint32_t tail;
	uint8_t data[ I2C_DATA_SIZE];
} t_i2cDataBuffer;


//---------------------------------------------------------------------------------

void TWI_init();
void TWI_isrTick();

t_i2cTransfer* TWI_putQueue( t_i2cTransfer* op);
t_i2cTransfer* TWI_fetchQueue();

//bool TWI_queue(uint8_t *buffer, uint32_t write_bytes);
//bool TWI_isBusy();
//bool TWI_post( uint8_t adr, uint8_t *buffer, uint32_t write_bytes);
void TWI_putOp( uint8_t addr, uint8_t dirBit, uint8_t* data, uint32_t len, void (*complete)( uint8_t* data, uint32_t len));
bool TWI_triggerSend();

uint8_t* TWI_malloc( uint8_t *buffer, uint32_t len);
bool TWI_free( uint8_t *buffer, uint32_t len);

void isrI2Cmaster();
void isrI2Cslave();

#endif /* I2C_TIVA_H_ */
