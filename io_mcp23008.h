/*
 * defines for mcp23008 i2c io expander
 */

#define MCP_ADDR			0x20

#define MCP_REG_IODIR		0x00			// 1d: input, 0:output
#define MCP_REG_IPOL		0x01			// 1: invert, 0d: normal
#define MCP_REG_GPINTEN		0x02			// 1: enable, 0d:disable
#define MCP_REG_DEFVAL		0x03			// 1, 0d
#define MCP_REG_INTCON		0x04			// 1: def val, 0d: prev val
#define MCP_REG_IOCON		0x05			// see bit desc
#define MCP_REG_GPPU		0x06			// 1: en, 0d: dis
#define MCP_REG_INTF		0x07			// read only
#define MCP_REG_INTCAP		0x08			// read only
#define MCP_REG_GPIO		0x09			// 0d, io register
#define MCP_REG_OLAT		0x0a			// 0d, output latch

#define MCP_IOCON_DISSEQOP	GPIO_PIN_5
#define MCP_IOCON_DISSLW	GPIO_PIN_4
//#define MCP_IOCON_HAEN		GPIO_PIN_3		// 23S08/spi only
#define MCP_IOCON_ODR		GPIO_PIN_2
#define MCP_IOCON_INTPOL	GPIO_PIN_1

#define MCP_PIN_RESET		7
#define MCP_PIN_SLEEP		6

#define MCP_MASK_INPUTS		0x3f
#define MCP_MASK_OUTPUTS	0xc0

/*
 * default register values
 *
 * MCP_IODIR		0x3f	// bits 7/6 out, 5-0 in
 * MCP_IPOL			0x00	// normal polarity
 * MCP_GPINTEN		0x3f	// enable int-on-change
 * MCP_DEFVAL		0x3f	// int def val
 * MCP_INTCON		0x3f	// compare against def val
 * MCP_IO_CON		0x00	// seqop on, slew on, opendrain off, int active low
 * MCP_GPPU			0x3f	// enable internal pullup on inputs
 * MCP_INTF			---		// read only
 * MCP_INTCAP		---		// ro, io captured when int triggered
 * MCP_GPIO			0xc0	// set outputs to high/inactive
 * MCP_OLAT			0xc0	// output latches
 */

void io_init();
void io_sendCmd( uint8_t addr, bool doWrite, uint8_t* data, uint32_t len, void (*complete)( uint8_t* data, uint32_t len));
void io_read_complete( uint8_t* data, uint32_t len);
void io_setIndex( uint8_t reg);
//void io_triggerReadInputs();
void io_isrFault();
void io_setSleep( bool sleep);
void io_resetFault();
