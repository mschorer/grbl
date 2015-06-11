/*
  serial.c - Low level functions for sending and recieving bytes via the serial port
  Part of Grbl v0.9

  Copyright (c) 2012-2014 Sungeun K. Jeon

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
/* 
  This file is based on work from Grbl v0.8, distributed under the 
  terms of the MIT-license. See COPYING for more details.  
    Copyright (c) 2009-2011 Simen Svale Skogsrud
    Copyright (c) 2011-2012 Sungeun K. Jeon
 */

//#include <avr/interrupt.h>
#include "system.h"
#include "serial.h"
#include "motion_control.h"
#include "protocol.h"

#include "hw_abstraction.h"
#include "cpu_map.h"


uint8_t serial_rx_buffer[RX_BUFFER_SIZE];
volatile uint8_t serial_rx_buffer_head = 0;
volatile uint8_t serial_rx_buffer_tail = 0;

uint8_t serial_tx_buffer[TX_BUFFER_SIZE];
volatile uint8_t serial_tx_buffer_head = 0;
volatile uint8_t serial_tx_buffer_tail = 0;

#define TX_IDLE	0
#define TX_XMIT	1

volatile uint8_t txState = TX_IDLE;


#ifdef ENABLE_XONXOFF
volatile uint8_t flow_ctrl = XON_SENT; // Flow control state variable
#endif


// Returns the number of bytes used in the RX serial buffer.
uint8_t serial_get_rx_buffer_count()
{
	uint8_t rtail = serial_rx_buffer_tail; // Copy to limit multiple calls to volatile
	if (serial_rx_buffer_head >= rtail) { return(serial_rx_buffer_head-rtail); }
	return (RX_BUFFER_SIZE - (rtail-serial_rx_buffer_head));
}


// Returns the number of bytes used in the TX serial buffer.
// NOTE: Not used except for debugging and ensuring no TX bottlenecks.
uint8_t serial_get_tx_buffer_count()
{
	uint8_t ttail = serial_tx_buffer_tail; // Copy to limit multiple calls to volatile
	if (serial_tx_buffer_head >= ttail) { return(serial_tx_buffer_head-ttail); }
	return (TX_BUFFER_SIZE - (ttail-serial_tx_buffer_head));
}

#ifdef CPU_MAP_TIVA

void isrUart();

//*****************************************************************************
//
// Variables tracking transmit and receive counts.
//
//*****************************************************************************
volatile uint32_t g_ui32UARTTxCount = 0;
volatile uint32_t g_ui32UARTRxCount = 0;
#ifdef DEBUG
uint32_t g_ui32UARTRxErrors = 0;
#endif

static bool g_bSendingBreak = false;
volatile uint32_t g_ui32SysTickCount = 0;

#define COMMAND_PACKET_RECEIVED 0x00000001
#define COMMAND_STATUS_UPDATE   0x00000002

volatile uint32_t g_ui32Flags = 0;
char *g_pcStatus;

#ifdef TIVA_SERIAL_UART

//*****************************************************************************
//static volatile bool g_bUSBConfigured = false;
static volatile bool txRunning = false;

void handleTransmit() {
	int32_t ui32rc;
	uint8_t tail = serial_tx_buffer_tail; // Temporary serial_tx_buffer_tail (to optimize for volatile)

	GPIO_WRITE_MASKED( STATUS_LED_PORT, 1<<STATUS_LED_BLUE, 1<<STATUS_LED_BLUE);

	while( UARTSpaceAvail( TIVA_SERIAL_UART)) {

#ifdef ENABLE_XONXOFF
		if (flow_ctrl == SEND_XOFF) {
			ui32rc = UARTCharPutNonBlocking(TIVA_SERIAL_UART, XOFF_CHAR);
			flow_ctrl = XOFF_SENT;
		} else if (flow_ctrl == SEND_XON) {
			ui32rc = UARTCharPutNonBlocking(TIVA_SERIAL_UART, XON_CHAR);
			flow_ctrl = XON_SENT;
		} else
#endif

		if ( tail == serial_tx_buffer_head) {
			// xmit buffer is empty
			// stop tx interrupts???
//			UARTIntDisable(TIVA_SERIAL_UART, UART_INT_TX);
			txRunning = false;

			break;
		}

		ui32rc = UARTCharPutNonBlocking(TIVA_SERIAL_UART, serial_tx_buffer[ tail]);

		if ( ui32rc == -1) break;

		tail++;
		if ( tail >= TX_BUFFER_SIZE) tail = 0;

		g_ui32UARTRxCount ++;
	}

	serial_tx_buffer_tail = tail;

	GPIO_WRITE_MASKED( STATUS_LED_PORT, 1<<STATUS_LED_BLUE, 0);
}

void handleReceive() {
	int32_t data;
	GPIO_WRITE_MASKED( STATUS_LED_PORT, 1<<STATUS_LED_GREEN,  1<<STATUS_LED_GREEN);

	while( UARTCharsAvail(TIVA_SERIAL_UART)) {
		g_ui32UARTTxCount++;

		uint8_t next_head;

		data = UARTCharGetNonBlocking(TIVA_SERIAL_UART);

		// Pick off runtime command characters directly from the serial stream. These characters are
		// not passed into the buffer, but these set system state flag bits for runtime execution.
		switch ( data) {

		case CMD_STATUS_REPORT: bit_true_atomic(sys.execute, EXEC_STATUS_REPORT); break; // Set as true
		case CMD_CYCLE_START:   bit_true_atomic(sys.execute, EXEC_CYCLE_START); break; // Set as true
		case CMD_FEED_HOLD:     bit_true_atomic(sys.execute, EXEC_FEED_HOLD); break; // Set as true
		case CMD_RESET:         mc_reset(); break; // Call motion control reset routine.

		default: // Write character to buffer
			next_head = serial_rx_buffer_head + 1;
			if (next_head == RX_BUFFER_SIZE) { next_head = 0; }

			// Write data to buffer unless it is full.
			if (next_head != serial_rx_buffer_tail) {
				serial_rx_buffer[serial_rx_buffer_head] = (uint8_t) data;
				serial_rx_buffer_head = next_head;

#ifdef ENABLE_XONXOFF
				if ((serial_get_rx_buffer_count() >= RX_BUFFER_FULL) && flow_ctrl == XON_SENT) {
					flow_ctrl = SEND_XOFF;

					// if the sender process is idle, tickle it
					if (serial_tx_buffer_head == serial_tx_buffer_tail) handleTransmit();
				}
#endif
			}
		}
	}

	GPIO_WRITE_MASKED( STATUS_LED_PORT, 1<<STATUS_LED_GREEN, 0);
}


//*****************************************************************************
//
// The UART interrupt handler.
//
//*****************************************************************************
void isrUart(void)
{
    uint32_t ui32Status;

    ui32Status = UARTIntStatus(TIVA_SERIAL_UART, true);
    UARTIntClear(TIVA_SERIAL_UART, ui32Status);

	if ( g_bSendingBreak) {
		return;
	}

	if ( ui32Status & ( UART_INT_RT | UART_INT_RX)) handleReceive();
	if ( ui32Status & UART_INT_TX) handleTransmit();
}

void serial_init() {

	//
	// Configure the required pins for USB operation.
	//
    SysCtlPeripheralEnable( TIVA_SERIAL_UART_PERI);
    SysCtlPeripheralEnable( TIVA_SERIAL_GPIO_PERI);
	SysCtlDelay(3);

    //
    // Set GPIO A0 and A1 as UART pins.
    //
    GPIOPinConfigure( TIVA_SERIAL_RX_PIN);
    GPIOPinConfigure( TIVA_SERIAL_TX_PIN);
    GPIOPinTypeUART( TIVA_SERIAL_PORT, TIVA_SERIAL_PINS);

    //
    // Configure the UART for 115,200, 8-N-1 operation.
    //
    UARTConfigSetExpClk(TIVA_SERIAL_UART, SysCtlClockGet(), 115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));

    UARTFIFOLevelSet( TIVA_SERIAL_UART, UART_FIFO_TX4_8, UART_FIFO_RX4_8);
    UARTFIFOEnable( TIVA_SERIAL_UART);

    UARTTxIntModeSet( TIVA_SERIAL_UART, UART_TXINT_MODE_FIFO);

    UARTIntRegister( TIVA_SERIAL_UART, isrUart);
    IntEnable(INT_UART0);
    UARTIntEnable(TIVA_SERIAL_UART, UART_INT_RX | UART_INT_RT | UART_INT_TX);
}

uint32_t serial_strLength( char *data) {
	uint32_t len = 0;

	while( *data++ != 0) {
		len++;
	}

	return len;
}

bool serial_sendString( char *data) {
	return serial_sendData( data, serial_strLength( data));
}

bool serial_sendData( char *data, uint32_t size) {
	uint32_t idx = 0;
	bool jumpStart;

//	UARTIntEnable(TIVA_SERIAL_UART, UART_INT_TX);

	uint8_t next_head = serial_tx_buffer_head;
	while( idx < size) {
//		serial_write( data[ idx++]);

		next_head++;
		if (next_head >= TX_BUFFER_SIZE) { next_head = 0; }

		// Wait until there is space in the buffer
		while (next_head == serial_tx_buffer_tail) {
			// TODO: Restructure st_prep_buffer() calls to be executed here during a long print.
			if (sys.execute & EXEC_RESET) { return false; } // Only check for abort to avoid an endless loop.
		}

		// Store data and advance head
		serial_tx_buffer[ serial_tx_buffer_head] = data[ idx++];
		jumpStart = (serial_tx_buffer_head == serial_tx_buffer_tail);
		serial_tx_buffer_head = next_head;

		// if not transmitting, push out first char
		if ( jumpStart) {
			handleTransmit();
		}
	}

	return false;
}


#else

//*****************************************************************************
static volatile bool g_bUSBConfigured = false;
static volatile bool usbTxRunning = false;



void serial_init() {

	//
	// Configure the required pins for USB operation.
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	g_bUSBConfigured = false;
	SysCtlDelay(3);

	GPIOPinTypeUSBAnalog(GPIO_PORTD_BASE, GPIO_PIN_5 | GPIO_PIN_4);

	USBBufferInit(&g_sTxBuffer);
	USBBufferInit(&g_sRxBuffer);

	USBStackModeSet(0, eUSBModeForceDevice, 0);

	USBDCDCInit(0, &g_sCDCDevice);
}

//*****************************************************************************
//
// This function is called whenever serial data is received from the UART.
// It is passed the accumulated error flags from each character received in
// this interrupt and determines from them whether or not an interrupt
// notification to the host is required.
//
// If a notification is required and the control interrupt endpoint is idle,
// we send the notification immediately.  If the endpoint is not idle, we
// accumulate the errors in a global variable which will be checked on
// completion of the previous notification and used to send a second one
// if necessary.
//
//*****************************************************************************
static void CheckForSerialStateChange(const tUSBDCDCDevice *psDevice, int32_t i32Errors) {
	uint16_t ui16SerialState;

	ui16SerialState = USB_CDC_SERIAL_STATE_TXCARRIER | USB_CDC_SERIAL_STATE_RXCARRIER;

	if(i32Errors) {
		if(i32Errors & UART_DR_OE) ui16SerialState |= USB_CDC_SERIAL_STATE_OVERRUN;
		if(i32Errors & UART_DR_PE) ui16SerialState |= USB_CDC_SERIAL_STATE_PARITY;
		if(i32Errors & UART_DR_FE) ui16SerialState |= USB_CDC_SERIAL_STATE_FRAMING;
		if(i32Errors & UART_DR_BE) ui16SerialState |= USB_CDC_SERIAL_STATE_BREAK;

		// Call the CDC driver to notify the state change.
		USBDCDCSerialStateChange((void *)psDevice, ui16SerialState);
	}
}

//*****************************************************************************
//
// transfer data to the xmit buffer, try to send blocks
//
// \return Returns UART error flags read during data reception.
//
//*****************************************************************************
static bool USBSend(void) {

	uint32_t ui32Space;
	uint8_t tail = serial_tx_buffer_tail; // Temporary serial_tx_buffer_tail (to optimize for volatile)
	uint8_t* data;
	bool rc = false;
	uint32_t size;

	GPIO_WRITE_MASKED( STATUS_LED_PORT, 1<<STATUS_LED_BLUE, 1<<STATUS_LED_BLUE);

	while( ui32Space = USBBufferSpaceAvailable((tUSBBuffer *)&g_sTxBuffer)) {

		if ( tail != serial_tx_buffer_head) {

			data = &serial_tx_buffer[tail];

			if ( tail < serial_tx_buffer_head) {
				size = serial_tx_buffer_head - tail;
			} else {
				size = TX_BUFFER_SIZE - tail;
			}
			if ( size > ui32Space) size = ui32Space;
			tail += size;
			tail %= TX_BUFFER_SIZE;

			USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, data, size);
			rc = true;

			g_ui32UARTRxCount += size;
		} else break;
	}
	serial_tx_buffer_tail = tail;
	usbTxRunning = rc;

	GPIO_WRITE_MASKED( STATUS_LED_PORT, 1<<STATUS_LED_BLUE, 0);

	return rc;
}

uint32_t serial_strLength( char *data) {
	uint32_t len = 0;

	while( *data++ != 0) {
		len++;
	}

	return len;
}

bool serial_sendString( char *data) {
	return serial_sendData( data, serial_strLength( data));
}

bool serial_sendData( char *data, uint32_t size) {
	uint32_t ui32Space = 0;

	if ( serial_tx_buffer_tail == serial_tx_buffer_head) {
		ui32Space = USBBufferSpaceAvailable((tUSBBuffer *)&g_sTxBuffer);

		if ( ui32Space > 0) {
			USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, (uint8_t *)data, (size < ui32Space) ? size : ui32Space);
		}
	}

	while( ui32Space < size) {
		serial_write( data[ ui32Space]);
		ui32Space++;
	}

	return false;
}

//*****************************************************************************
//
// Take as many bytes from the transmit buffer as we have space for and move
// them into the USB UART's transmit FIFO.
//
//*****************************************************************************
static void USBRead() {
	uint8_t data;
	uint32_t iRead;

	if ( g_bSendingBreak) {
		return;
	}

	GPIO_WRITE_MASKED( STATUS_LED_PORT, 1<<STATUS_LED_GREEN,  1<<STATUS_LED_GREEN);

	while( iRead = USBBufferRead((tUSBBuffer *)&g_sRxBuffer, &data, 1)) {
		g_ui32UARTTxCount++;

		uint8_t next_head;

		// Pick off runtime command characters directly from the serial stream. These characters are
		// not passed into the buffer, but these set system state flag bits for runtime execution.
		switch (data) {

		case CMD_STATUS_REPORT: bit_true_atomic(sys.execute, EXEC_STATUS_REPORT); break; // Set as true
		case CMD_CYCLE_START:   bit_true_atomic(sys.execute, EXEC_CYCLE_START); break; // Set as true
		case CMD_FEED_HOLD:     bit_true_atomic(sys.execute, EXEC_FEED_HOLD); break; // Set as true
		case CMD_RESET:         mc_reset(); break; // Call motion control reset routine.

		default: // Write character to buffer
			next_head = serial_rx_buffer_head + 1;
			if (next_head == RX_BUFFER_SIZE) { next_head = 0; }

			// Write data to buffer unless it is full.
			if (next_head != serial_rx_buffer_tail) {
				serial_rx_buffer[serial_rx_buffer_head] = data;
				serial_rx_buffer_head = next_head;

#ifdef ENABLE_XONXOFF
				if ((serial_get_rx_buffer_count() >= RX_BUFFER_FULL) && flow_ctrl == XON_SENT) {
					flow_ctrl = SEND_XOFF;
					UCSR0B |=  (1 << UDRIE0); // Force TX
				}
#endif

			}
		}
	}
	GPIO_WRITE_MASKED( STATUS_LED_PORT, 1<<STATUS_LED_GREEN, 0);
}


//*****************************************************************************
//
// Set the state of the RS232 RTS and DTR signals.
//
//*****************************************************************************
static void SetControlLineState(uint16_t ui16State) {
	//
	// TODO: If configured with GPIOs controlling the handshake lines,
	// set them appropriately depending upon the flags passed in the wValue
	// field of the request structure passed.
	//
	;
}
//*****************************************************************************
//
// This function sets or clears a break condition on the redirected UART RX
// line.  A break is started when the function is called with \e bSend set to
// \b true and persists until the function is called again with \e bSend set
// to \b false.
//
//*****************************************************************************
static void
SendBreak(bool bSend)
{
	//
	// Are we being asked to start or stop the break condition?
	//
	/*
    if(!bSend) {
        UARTBreakCtl(USB_UART_BASE, false);
        g_bSendingBreak = false;
    } else {
        UARTBreakCtl(USB_UART_BASE, true);
        g_bSendingBreak = true;
    }
	 */
}

//*****************************************************************************
//
// Handles CDC driver notifications related to control and setup of the device.
//
// \param pvCBData is the client-supplied callback pointer for this channel.
// \param ui32Event identifies the event we are being notified about.
// \param ui32MsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the CDC driver to perform control-related
// operations on behalf of the USB host.  These functions include setting
// and querying the serial communication parameters, setting handshake line
// states and sending break conditions.
//
// \return The return value is event-specific.
//
//*****************************************************************************
uint32_t
ControlHandler(void *pvCBData, uint32_t ui32Event,
		uint32_t ui32MsgValue, void *pvMsgData)
{
	uint32_t ui32IntsOff;

	switch(ui32Event) {
	//
	// We are connected to a host and communication is now possible.
	//
	case USB_EVENT_CONNECTED:
		g_bUSBConfigured = true;

		//
		// Flush our buffers.
		//
		USBBufferFlush(&g_sTxBuffer);
		USBBufferFlush(&g_sRxBuffer);

		//
		// Tell the main loop to update the display.
		//
		ui32IntsOff = IntMasterDisable();
		g_pcStatus = "Connected";
		g_ui32Flags |= COMMAND_STATUS_UPDATE;
		if(!ui32IntsOff)
		{
			IntMasterEnable();
		}
		break;

		//
		// The host has disconnected.
		//
	case USB_EVENT_DISCONNECTED:
		g_bUSBConfigured = false;
		ui32IntsOff = IntMasterDisable();
		g_pcStatus = "Disconnected";
		g_ui32Flags |= COMMAND_STATUS_UPDATE;
		if(!ui32IntsOff)
		{
			IntMasterEnable();
		}
		break;

		//
		// Return the current serial communication parameters.
		//
	case USBD_CDC_EVENT_GET_LINE_CODING:
//		GetLineCoding(pvMsgData);
		break;

		//
		// Set the current serial communication parameters.
		//
	case USBD_CDC_EVENT_SET_LINE_CODING:
//		SetLineCoding(pvMsgData);
		break;

		//
		// Set the current serial communication parameters.
		//
	case USBD_CDC_EVENT_SET_CONTROL_LINE_STATE:
		SetControlLineState((uint16_t)ui32MsgValue);
		break;

		//
		// Send a break condition on the serial line.
		//
	case USBD_CDC_EVENT_SEND_BREAK:
		SendBreak(true);
		break;

		//
		// Clear the break condition on the serial line.
		//
	case USBD_CDC_EVENT_CLEAR_BREAK:
		SendBreak(false);
		break;

		//
		// Ignore SUSPEND and RESUME for now.
		//
	case USB_EVENT_SUSPEND:
	case USB_EVENT_RESUME:
		break;

		//
		// We don't expect to receive any other events.  Ignore any that show
		// up in a release build or hang in a debug build.
		//
	default:
#ifdef DEBUG
		while(1);
#else
		break;
#endif

	}

	return(0);
}

//*****************************************************************************
//
// Handles CDC driver notifications related to the transmit channel (data to
// the USB host).
//
// \param ui32CBData is the client-supplied callback pointer for this channel.
// \param ui32Event identifies the event we are being notified about.
// \param ui32MsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the CDC driver to notify us of any events
// related to operation of the transmit data channel (the IN channel carrying
// data to the USB host).
//
// \return The return value is event-specific.
//
//*****************************************************************************
uint32_t TxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue, void *pvMsgData) {

	switch(ui32Event) {
	case USB_EVENT_TX_COMPLETE:
		usbTxRunning = false;
		//
		// Since we are using the USBBuffer, we don't need to do anything
		// here.
		//
		USBSend();
		break;

		//
		// We don't expect to receive any other events.  Ignore any that show
		// up in a release build or hang in a debug build.
		//
	default:
#ifdef DEBUG
		while(1);
#else
		break;
#endif

	}
	return(0);
}

//*****************************************************************************
//
// Handles CDC driver notifications related to the receive channel (data from
// the USB host).
//
// \param ui32CBData is the client-supplied callback data value for this channel.
// \param ui32Event identifies the event we are being notified about.
// \param ui32MsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the CDC driver to notify us of any events
// related to operation of the receive data channel (the OUT channel carrying
// data from the USB host).
//
// \return The return value is event-specific.
//
//*****************************************************************************
uint32_t RxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue, void *pvMsgData) {

	uint32_t ui32Count;

	switch(ui32Event) {
	//
	// A new packet has been received.
	//
	case USB_EVENT_RX_AVAILABLE:
		//
		// Feed some characters into the UART TX FIFO and enable the
		// interrupt so we are told when there is more space.
		//
		USBRead();
		break;

		//
		// We are being asked how much unprocessed data we have still to
		// process. We return 0 if the UART is currently idle or 1 if it is
		// in the process of transmitting something. The actual number of
		// bytes in the UART FIFO is not important here, merely whether or
		// not everything previously sent to us has been transmitted.
		//
	case USB_EVENT_DATA_REMAINING:
		//
		// Get the number of bytes in the buffer and add 1 if some data
		// still has to clear the transmitter.
		//
		//ui32Count = serial_tx_UARTBusy(USB_UART_BASE) ? 1 : 0;

		ui32Count = ( TX_BUFFER_SIZE + serial_tx_buffer_head - serial_tx_buffer_tail) % TX_BUFFER_SIZE;
		return(ui32Count);

		//
		// We are being asked to provide a buffer into which the next packet
		// can be read. We do not support this mode of receiving data so let
		// the driver know by returning 0. The CDC driver should not be sending
		// this message but this is included just for illustration and
		// completeness.
		//
	case USB_EVENT_REQUEST_BUFFER:
		return(0);

		//
		// We don't expect to receive any other events.  Ignore any that show
		// up in a release build or hang in a debug build.
		//
	default:
#ifdef DEBUG
		while(1);
#else
		break;
#endif
	}

	return(0);
}
#endif

#else

void serial_init()
{
	// Set baud rate
#if BAUD_RATE < 57600
	uint16_t UBRR0_value = ((F_CPU / (8L * BAUD_RATE)) - 1)/2 ;
	UCSR0A &= ~(1 << U2X0); // baud doubler off  - Only needed on Uno XXX
#else
	uint16_t UBRR0_value = ((F_CPU / (4L * BAUD_RATE)) - 1)/2;
	UCSR0A |= (1 << U2X0);  // baud doubler on for high baud rates, i.e. 115200
#endif
	UBRR0H = UBRR0_value >> 8;
	UBRR0L = UBRR0_value;

	// enable rx and tx
	UCSR0B |= 1<<RXEN0;
	UCSR0B |= 1<<TXEN0;

	// enable interrupt on complete reception of a byte
	UCSR0B |= 1<<RXCIE0;

	// defaults to 8-bit, no parity, 1 stop bit
}

void serial_tx_trigger() {
	UCSR0B |=  (1 << UDRIE0);
}


// Data Register Empty Interrupt handler
ISR(SERIAL_UDRE)
{
	uint8_t tail = serial_tx_buffer_tail; // Temporary serial_tx_buffer_tail (to optimize for volatile)

#ifdef ENABLE_XONXOFF
	if (flow_ctrl == SEND_XOFF) {
		UDR0 = XOFF_CHAR;
		flow_ctrl = XOFF_SENT;
	} else if (flow_ctrl == SEND_XON) {
		UDR0 = XON_CHAR;
		flow_ctrl = XON_SENT;
	} else
#endif
	{
		// Send a byte from the buffer
		UDR0 = serial_tx_buffer[tail];

		// Update tail position
		tail++;
		if (tail == TX_BUFFER_SIZE) { tail = 0; }

		serial_tx_buffer_tail = tail;
	}

	// Turn off Data Register Empty Interrupt to stop tx-streaming if this concludes the transfer
	if (tail == serial_tx_buffer_head) { UCSR0B &= ~(1 << UDRIE0); }
}

ISR(SERIAL_RX)
{
	uint8_t data = UDR0;
	uint8_t next_head;

	// Pick off runtime command characters directly from the serial stream. These characters are
	// not passed into the buffer, but these set system state flag bits for runtime execution.
	switch (data) {
	case CMD_STATUS_REPORT: bit_true_atomic(sys.execute, EXEC_STATUS_REPORT); break; // Set as true
	case CMD_CYCLE_START:   bit_true_atomic(sys.execute, EXEC_CYCLE_START); break; // Set as true
	case CMD_FEED_HOLD:     bit_true_atomic(sys.execute, EXEC_FEED_HOLD); break; // Set as true
	case CMD_RESET:         mc_reset(); break; // Call motion control reset routine.
	default: // Write character to buffer
		next_head = serial_rx_buffer_head + 1;
		if (next_head == RX_BUFFER_SIZE) { next_head = 0; }

		// Write data to buffer unless it is full.
		if (next_head != serial_rx_buffer_tail) {
			serial_rx_buffer[serial_rx_buffer_head] = data;
			serial_rx_buffer_head = next_head;

#ifdef ENABLE_XONXOFF
			if ((serial_get_rx_buffer_count() >= RX_BUFFER_FULL) && flow_ctrl == XON_SENT) {
				flow_ctrl = SEND_XOFF;
				UCSR0B |=  (1 << UDRIE0); // Force TX
			}
#endif

		}
		//TODO: else alarm on overflow?
	}
}

#endif


// Writes one byte to the TX serial buffer. Called by main program.
// TODO: Check if we can speed this up for writing strings, rather than single bytes.
void serial_write(uint8_t data) {
	// Calculate next head
	uint8_t next_head = serial_tx_buffer_head + 1;
	if (next_head == TX_BUFFER_SIZE) { next_head = 0; }

	// Wait until there is space in the buffer
	while (next_head == serial_tx_buffer_tail) {
		// TODO: Restructure st_prep_buffer() calls to be executed here during a long print.
		if (sys.execute & EXEC_RESET) { return; } // Only check for abort to avoid an endless loop.
	}

	// Store data and advance head
	serial_tx_buffer[serial_tx_buffer_head] = data;
	serial_tx_buffer_head = next_head;

	// Enable Data Register Empty Interrupt to make sure tx-streaming is running
//	if ( data == '\n') USBSend();
}

// Fetches the first byte in the serial read buffer. Called by main program.
uint8_t serial_read()
{
	uint8_t tail = serial_rx_buffer_tail; // Temporary serial_rx_buffer_tail (to optimize for volatile)
	if (serial_rx_buffer_head == tail) {
		return SERIAL_NO_DATA;
	} else {
		uint8_t data = serial_rx_buffer[tail];

		tail++;
		if (tail == RX_BUFFER_SIZE) { tail = 0; }
		serial_rx_buffer_tail = tail;

#ifdef ENABLE_XONXOFF
		if ((serial_get_rx_buffer_count() < RX_BUFFER_LOW) && flow_ctrl == XOFF_SENT) {
			flow_ctrl = SEND_XON;

#ifdef CPU_MAP_TIVA
			// if the sender process is idle, tickle it
			if (serial_tx_buffer_head == serial_tx_buffer_tail) handleTransmit();
#else
			UCSR0B |=  (1 << UDRIE0); // Force TX
#endif
		}
#endif

		return data;
	}
}


void serial_reset_read_buffer() 
{
	serial_rx_buffer_tail = serial_rx_buffer_head;

#ifdef ENABLE_XONXOFF
	flow_ctrl = XON_SENT;
#endif
}
