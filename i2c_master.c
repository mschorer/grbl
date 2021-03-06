/*
  i2c_master.c - provide interrupt driven, buffered i2c transfers for atmel 328p processors
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

#include <inttypes.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
//#include <avr/io.h>
//#include <avr/interrupt.h>
//#include <avr/pgmspace.h>
#include "cpu_map.h"

#ifndef CPU_MAP_TIVA

#include "i2c_master.h"

// initialize the Master TWI, uses included parameters from twim.h
void TWI_init(){
  //SCL_CLOCK and transfer rate set in twim.h
  /* initialize TWI clock: TWPS = 0 => prescaler = 1 */
  TWCR = (TWI_ACK);
  TWSR = (0<<TWPS1) | (0<<TWPS0);	/* no prescaler */
  TWBR = ((F_CPU/SCL_CLOCK)-16)/2;	/* must be > 10 for stable operation */
  TWI_busy=0;
}

uint8_t TWI_tick(){
	if ( TWI_busy) TWI_busy++;
	
	if ( TWI_busy > TWI_BUS_BLOCK_TICKS) TWI_busy = 0;
	
	return TWI_busy;
}

// master write to slave
void TWI_master_start_write(uint8_t slave_addr, uint16_t write_bytes) {	//7 bit slave address, number of bytes to write
    TWI_busy=1;
	if(write_bytes>TWI_BUFFER_WR_MAX){
        TWI_write_bytes=TWI_BUFFER_WR_MAX;
    }else{
        TWI_write_bytes=write_bytes;
    }
    TWI_operation=TWI_OP_WRITE_ONLY;
    TWI_master_state = TWI_WRITE_STATE;
    TWI_target_slave_addr = slave_addr;
    TWCR = TWI_START; // start TWI master mode
}

// master read from slave
void TWI_master_start_read(uint8_t slave_addr, uint16_t read_bytes){
    TWI_busy=1;
    if(read_bytes>TWI_BUFFER_RD_MAX){
        TWI_read_bytes=TWI_BUFFER_RD_MAX;
    }else{
        TWI_read_bytes=read_bytes;
    }
    TWI_operation=TWI_OP_READ_ONLY;
    TWI_master_state = TWI_READ_STATE;
    TWI_target_slave_addr = slave_addr;
    TWCR = TWI_START; // start TWI master mode
}

// master write then read without releasing buss between
void TWI_master_start_write_then_read(uint8_t slave_addr, uint16_t write_bytes, uint16_t read_bytes){
    TWI_busy=1;
    if(write_bytes>TWI_BUFFER_WR_MAX){
        TWI_write_bytes=TWI_BUFFER_WR_MAX;
    }else{
        TWI_write_bytes=write_bytes;
    }
    if(read_bytes>TWI_BUFFER_RD_MAX){
        TWI_read_bytes=TWI_BUFFER_RD_MAX;
    }else{
        TWI_read_bytes=read_bytes;
    }
    TWI_operation=TWI_OP_WRITE_THEN_READ;
    TWI_master_state = TWI_WRITE_STATE;
    TWI_target_slave_addr = slave_addr;
    TWCR = TWI_START; // start TWI master mode 
}

// Routine to service interrupts from the TWI hardware.
// The most important thing is that this routine runs fast and returns control
// to the hardware asap. 
// See pages 229, 232, 235, and 238 of the ATmega328 datasheet for detailed 
// explanation of the logic below.
ISR(TWI_vect){

    TWI_status = TWSR & TWI_TWSR_status_mask;
    switch(TWI_status){
        case TWI_repeated_start_sent:
        case TWI_start_sent:
        	switch(TWI_master_state){
        		case TWI_WRITE_STATE:
        			TWI_buffer_pos=0; // point to 1st byte
        			TWDR = (TWI_target_slave_addr<<1) | 0x00; // set SLA_W
        		break;
        		case TWI_READ_STATE:
        			TWI_buffer_pos=0; // point to first byte
        			TWDR = (TWI_target_slave_addr<<1) | 0x01; // set SLA_R
        		break;
            }
            TWCR = TWI_ACK; // transmit
        break;

        case TWI_SLA_W_sent_ack_received:   
        case TWI_data_sent_ack_received:

            if(TWI_buffer_pos==TWI_write_bytes){
                if(TWI_operation==TWI_OP_WRITE_THEN_READ){
                    TWI_master_state=TWI_READ_STATE; // now read from slave
                    TWCR = TWI_START; // transmit repeated start
                }else{
                    TWCR = TWI_STOP; // release the buss

                    // when TWSTO goes low, the stop condition is sent
                    // but this doesn't really matter, because if we start another transfer,
                    // the first interrupt will happen after start has been transmitted
                    // while(TWCR & (1<<TWSTO)); // wait for it
                    TWI_busy=0;
                }
            }else{ 
                TWDR = TWI_buffer_out[TWI_buffer_pos++]; // load data
                TWCR = TWI_ENABLE; // transmit
            }
            break;

        case TWI_data_received_ack_returned:
            TWI_buffer_in[TWI_buffer_pos++]=TWDR; // save byte

        case TWI_SLA_R_sent_ack_received: 
            if(TWI_buffer_pos==(TWI_read_bytes-1)){
                TWCR = TWI_NACK; // get last byte then nack
            }else{
                TWCR = TWI_ACK; // get next byte then ack
            }
            break;

        case TWI_data_received_nack_returned:            
            TWI_buffer_in[TWI_buffer_pos++]=TWDR; // save byte
            TWCR = TWI_STOP; // release the buss
            //while(TWCR & (1<<TWSTO)); // wait for it**
            TWI_busy=0;
            break;

        case TWI_data_sent_nack_received:
        case TWI_SLA_R_sent_nack_received:
        case TWI_arbitration_lost:
        default:
            TWCR=TWI_STOP;

            // when TWSTO goes low, the stop condition is sent
            // but this doesn't really matter, because if we start another transfer,
            // the first interrupt will happen after start has been transmitted
            // while(TWCR & (1<<TWSTO)); // wait for it***

            TWCR=TWI_START; // try again
            break;
    }
}

#endif // CPU_MAP_TIVA
