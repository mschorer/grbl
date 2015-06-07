/*
  i2c_master.h - provide interrupt driven, buffered i2c transfers for atmel 328p processors
  Part of Grbl v0.9

  Integrated by (i) 2014 ms@ms-ite.de  

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
// interrupt driven i2c master based on TWI hardware

#include "cpu_map.h"


#ifndef CPU_MAP_TIVA

#define SCL_CLOCK  400000L

#define TWI_BUS_BLOCK_TICKS 2
//------------------------------------------------------------- TWI ------
#define TWI_TWSR_status_mask 0xF8

// Status codes for TWI Master Mode (TWSR)
#define TWI_start_sent 0x08
#define TWI_repeated_start_sent 0x10
#define TWI_arbitration_lost 0x38

// Status codes for TWI Master Transmitter Mode 
#define TWI_SLA_W_sent_ack_received 0x18
#define TWI_SLA_W_sent_nack_received 0x20
#define TWI_data_sent_ack_received 0x28
#define TWI_data_sent_nack_received 0x30

// Status codes for TWI Master Receiver Mode
#define TWI_SLA_R_sent_ack_received 0x40
#define TWI_SLA_R_sent_nack_received 0x48
#define TWI_data_received_ack_returned 0x50
#define TWI_data_received_nack_returned 0x58

#define TWI_BUFFER_RD_MAX 2
volatile uint8_t TWI_buffer_in[TWI_BUFFER_RD_MAX];

#define TWI_BUFFER_WR_MAX 42
volatile uint8_t TWI_buffer_out[TWI_BUFFER_WR_MAX];

volatile uint8_t TWI_target_slave_addr;

//uint8_t j =0;

volatile uint8_t TWI_status;
#define TWI_WRITE_STATE 0x01
#define TWI_READ_STATE 0x02
//volatile uint8_t TWI_operation;

// call types
volatile uint8_t TWI_master_state;
#define TWI_OP_WRITE_ONLY 0x01
#define TWI_OP_READ_ONLY 0x02
#define TWI_OP_WRITE_THEN_READ 0x03

// control variables
volatile uint8_t TWI_operation;
volatile uint8_t TWI_busy;
volatile uint8_t TWI_error;

// buffers and variables
//volatile uint16_t TWI_buffer_max;
volatile uint16_t TWI_buffer_pos;
//volatile uint8_t TWI_buffer_len;
volatile uint16_t TWI_read_bytes;
volatile uint16_t TWI_write_bytes;

#define TWI_ENABLE  _BV(TWEN) | _BV(TWINT) | _BV(TWIE)
#define TWI_ACK     _BV(TWEA)  |    TWI_ENABLE
#define TWI_NACK                    TWI_ENABLE
#define TWI_START   _BV(TWSTA) |    TWI_ENABLE
#define TWI_STOP    _BV(TWSTO) |    TWI_ENABLE

void TWI_init();
uint8_t TWI_tick();

void TWI_queue(uint8_t *buf, uint8_t write_bytes);
void TWI_send( uint8_t adr);

void TWI_master_start_write(uint8_t slave_addr, uint16_t write_bytes);
void TWI_master_start_read(uint8_t slave_addr, uint16_t read_bytes);
void TWI_master_start_write_then_read(uint8_t slave_addr, uint16_t write_bytes, uint16_t read_bytes);

#endif // CPU_MAP_TIVA
