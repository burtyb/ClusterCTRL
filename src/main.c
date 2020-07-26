/* Name: main.c
 * Project: i2c-tiny-usb-clusterhat
 * Author: Till Harbaum (i2c-tiny-usb)
 * Author: Chris Burton (Cluster CTRL)
 * Tabsize: 4
 * Copyright: (c) 2005 by Till Harbaum <till@harbaum.org>
 * Copyright: (c) 2019-2020 by Chris Burton <chris@8086.net>
 * License: GPL
 */

#include <stdio.h>
#include <ctype.h>
#include <string.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>

#include <util/delay.h>

#include "config.h" // Include hardware specific configuration header

// CTRL_SIZE = number of bytes needed to store one bit per node
# if CTRL_MAXPI%8
# define CTRL_SIZE ((CTRL_MAXPI/8)+1)
# else
# define CTRL_SIZE (CTRL_MAXPI/8)
# endif

// Bit manipulation helpers
#define P2BIT(x) ((x)%8)
#define P2BYTE(x) ((x)/8)

#ifndef CTRL_TYPE
# error "Controller type and pin mapping must be defined"
#endif

struct {
	unsigned char enpwr;
#if defined(LEDACTPORT)
	unsigned char enact;
#endif
	unsigned char enp[CTRL_SIZE];
	unsigned char enpled[CTRL_SIZE]; 
#if defined(USBBOOT)
	unsigned char usbboot[CTRL_SIZE];
#endif
} state;

#if defined(UARTDEBUG) && defined(DEBUG)
#define DEBUGF(format, args...) printf_P(PSTR(format), ##args)

static int uart_putchar(char c, FILE *stream) {
  if (c == '\n')
    uart_putchar('\r', stream);
  loop_until_bit_is_set(UCSR0A, UDRE0);
  UDR0 = c;
  return 0;
}

static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL,
					 _FDEV_SETUP_WRITE);
#else
# define DEBUGF(format, args...)
# undef DEBUG
# undef UARTDEBUG
#endif


// use avrusb library
#include "usbdrv.h"
#include "oddebug.h"

#define ENABLE_SCL_EXPAND

/* commands from USB, must e.g. match command ids in kernel driver */
#define CMD_ECHO       0
#define CMD_GET_FUNC   1
#define CMD_SET_DELAY  2
#define CMD_GET_STATUS 3

#define CMD_I2C_IO     4
#define CMD_I2C_BEGIN  1  // flag fo I2C_IO
#define CMD_I2C_END    2  // flag fo I2C_IO
/* Above is 4/5/6 when or'd */

/* linux kernel flags */
#define I2C_M_TEN		0x10	/* we have a ten bit chip address */
#define I2C_M_RD		0x01
#define I2C_M_NOSTART		0x4000
#define I2C_M_REV_DIR_ADDR	0x2000
#define I2C_M_IGNORE_NAK	0x1000
#define I2C_M_NO_RD_ACK		0x0800

/* To determine what functionality is present */
#define I2C_FUNC_I2C			0x00000001
#define I2C_FUNC_10BIT_ADDR		0x00000002
#define I2C_FUNC_PROTOCOL_MANGLING	0x00000004 /* I2C_M_{REV_DIR_ADDR,NOSTART,..} */
#define I2C_FUNC_SMBUS_HWPEC_CALC	0x00000008 /* SMBus 2.0 */
#define I2C_FUNC_SMBUS_READ_WORD_DATA_PEC  0x00000800 /* SMBus 2.0 */ 
#define I2C_FUNC_SMBUS_WRITE_WORD_DATA_PEC 0x00001000 /* SMBus 2.0 */ 
#define I2C_FUNC_SMBUS_PROC_CALL_PEC	0x00002000 /* SMBus 2.0 */
#define I2C_FUNC_SMBUS_BLOCK_PROC_CALL_PEC 0x00004000 /* SMBus 2.0 */
#define I2C_FUNC_SMBUS_BLOCK_PROC_CALL	0x00008000 /* SMBus 2.0 */
#define I2C_FUNC_SMBUS_QUICK		0x00010000 
#define I2C_FUNC_SMBUS_READ_BYTE	0x00020000 
#define I2C_FUNC_SMBUS_WRITE_BYTE	0x00040000 
#define I2C_FUNC_SMBUS_READ_BYTE_DATA	0x00080000 
#define I2C_FUNC_SMBUS_WRITE_BYTE_DATA	0x00100000 
#define I2C_FUNC_SMBUS_READ_WORD_DATA	0x00200000 
#define I2C_FUNC_SMBUS_WRITE_WORD_DATA	0x00400000 
#define I2C_FUNC_SMBUS_PROC_CALL	0x00800000 
#define I2C_FUNC_SMBUS_READ_BLOCK_DATA	0x01000000 
#define I2C_FUNC_SMBUS_WRITE_BLOCK_DATA 0x02000000 
#define I2C_FUNC_SMBUS_READ_I2C_BLOCK	0x04000000 /* I2C-like block xfer  */
#define I2C_FUNC_SMBUS_WRITE_I2C_BLOCK	0x08000000 /* w/ 1-byte reg. addr. */
#define I2C_FUNC_SMBUS_READ_I2C_BLOCK_2	 0x10000000 /* I2C-like block xfer  */
#define I2C_FUNC_SMBUS_WRITE_I2C_BLOCK_2 0x20000000 /* w/ 2-byte reg. addr. */
#define I2C_FUNC_SMBUS_READ_BLOCK_DATA_PEC  0x40000000 /* SMBus 2.0 */
#define I2C_FUNC_SMBUS_WRITE_BLOCK_DATA_PEC 0x80000000 /* SMBus 2.0 */

#define I2C_FUNC_SMBUS_BYTE I2C_FUNC_SMBUS_READ_BYTE | \
                            I2C_FUNC_SMBUS_WRITE_BYTE
#define I2C_FUNC_SMBUS_BYTE_DATA I2C_FUNC_SMBUS_READ_BYTE_DATA | \
                                 I2C_FUNC_SMBUS_WRITE_BYTE_DATA
#define I2C_FUNC_SMBUS_WORD_DATA I2C_FUNC_SMBUS_READ_WORD_DATA | \
                                 I2C_FUNC_SMBUS_WRITE_WORD_DATA
#define I2C_FUNC_SMBUS_BLOCK_DATA I2C_FUNC_SMBUS_READ_BLOCK_DATA | \
                                  I2C_FUNC_SMBUS_WRITE_BLOCK_DATA
#define I2C_FUNC_SMBUS_I2C_BLOCK I2C_FUNC_SMBUS_READ_I2C_BLOCK | \
                                  I2C_FUNC_SMBUS_WRITE_I2C_BLOCK

#define I2C_FUNC_SMBUS_EMUL I2C_FUNC_SMBUS_QUICK | \
                            I2C_FUNC_SMBUS_BYTE | \
                            I2C_FUNC_SMBUS_BYTE_DATA | \
                            I2C_FUNC_SMBUS_WORD_DATA | \
                            I2C_FUNC_SMBUS_PROC_CALL | \
                            I2C_FUNC_SMBUS_WRITE_BLOCK_DATA | \
                            I2C_FUNC_SMBUS_WRITE_BLOCK_DATA_PEC | \
                            I2C_FUNC_SMBUS_I2C_BLOCK

/* the currently support capability is quite limited */
const unsigned long func PROGMEM = I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;

/* ------------------------------------------------------------------------- */
#define DEFAULT_DELAY 10  // default 10us (100khz)
static unsigned short clock_delay  = DEFAULT_DELAY;
static unsigned short clock_delay2 = DEFAULT_DELAY/2;

static unsigned short expected;
static unsigned char saved_cmd;
static unsigned short saved_addr;

#if defined(I2C_HW)

static void i2c_io_set_sda(uchar hi) {
  if(hi) {
    I2C_DDR  &= ~I2C_SDA;    // high -> input
    I2C_PORT |=  I2C_SDA;    // with pullup
  } else {
    I2C_DDR  |=  I2C_SDA;    // low -> output
    I2C_PORT &= ~I2C_SDA;    // drive low
  }
}

static uchar i2c_io_get_sda(void) {
  return(I2C_PIN & I2C_SDA);
}

static void i2c_io_set_scl(uchar hi) {
# if defined(ENABLE_SCL_EXPAND)
  _delay_loop_2(clock_delay2);
  if(hi) {
    I2C_DDR &= ~I2C_SCL;          // port is input
    I2C_PORT |= I2C_SCL;          // enable pullup

    // wait while pin is pulled low by client
    while(!(I2C_PIN & I2C_SCL));
  } else {
    I2C_DDR |= I2C_SCL;           // port is output
    I2C_PORT &= ~I2C_SCL;         // drive it low
  }
  _delay_loop_2(clock_delay);
# else
  _delay_loop_2(clock_delay2);
  if(hi) I2C_PORT |=  I2C_SCL;    // port is high
  else   I2C_PORT &= ~I2C_SCL;    // port is low
  _delay_loop_2(clock_delay);
# endif
}

static void i2c_init(void) {
# if defined(DEBUGI2C)
  DEBUGF("INIT I2C\n");
# endif
  /* init the sda/scl pins */
  I2C_DDR &= ~I2C_SDA;            // port is input
  I2C_PORT |= I2C_SDA;            // enable pullup
# if defined(ENABLE_SCL_EXPAND)
  I2C_DDR &= ~I2C_SCL;            // port is input
  I2C_PORT |= I2C_SCL;            // enable pullup
# else
  I2C_DDR |= I2C_SCL;             // port is output
# endif

  /* no bytes to be expected */
  expected = 0;
}
/* clock HI, delay, then LO */
static void i2c_scl_toggle(void) {
  i2c_io_set_scl(1);
  i2c_io_set_scl(0);
}

/* i2c start condition */
static void i2c_start(void) {
# if defined(DEBUGI2C)
  DEBUGF("START\n");
# endif
  PORTB |= (1<<PB2); /* ACT LED ON */
  i2c_io_set_sda(0);
  i2c_io_set_scl(0);
}

# if defined(NEEDI2CREPSTART)
/* i2c repeated start condition */
static void i2c_repstart(void)
{
#  if defined(DEBUGI2C)
  DEBUGF("REPSTART\n");
#  endif
  /* scl, sda may not be high */
  i2c_io_set_sda(1);
  i2c_io_set_scl(1);

  i2c_io_set_sda(0);
  i2c_io_set_scl(0);
}
# endif

/* i2c stop condition */
void i2c_stop(void) {
# if defined(DEBUGI2C)
  DEBUGF("STOP\n");
# endif
  i2c_io_set_sda(0);
  i2c_io_set_scl(1);
  i2c_io_set_sda(1);
  PORTB &= ~(1<<PB2); /* LED OFF */
}

uchar i2c_put_u08(uchar b) {
  char i;
# if defined(DEBUGI2C)
  DEBUGF("PUT:%d\n", b);
# endif

  for (i=7;i>=0;i--) {
    if ( b & (1<<i) )  i2c_io_set_sda(1);
    else               i2c_io_set_sda(0);

    i2c_scl_toggle();           // clock HI, delay, then LO
  }

  i2c_io_set_sda(1);            // leave SDL HI
  i2c_io_set_scl(1);            // clock back up

  b = i2c_io_get_sda();         // get the ACK bit
  i2c_io_set_scl(0);            // not really ??

  return(b == 0);               // return ACK value
}


uchar i2c_get_u08(uchar last) {
  char i;
  uchar c,b = 0;
# if defined(DEBUGI2C)
  DEBUGF("GET:%d\n", last);
# endif
  i2c_io_set_sda(1);            // make sure pullups are activated
  i2c_io_set_scl(0);            // clock LOW

  for(i=7;i>=0;i--) {
    i2c_io_set_scl(1);          // clock HI
    c = i2c_io_get_sda();
    b <<= 1;
    if(c) b |= 1;
    i2c_io_set_scl(0);          // clock LO
  }

  if(last) i2c_io_set_sda(1);   // set NAK
  else     i2c_io_set_sda(0);   // set ACK

  i2c_scl_toggle();             // clock pulse
  i2c_io_set_sda(1);            // leave with SDL HI

  return b;                     // return received byte
}

/* Is there an I2C device at this address? 
 * returns 0 if not found
*/

uchar i2c_exists(uchar addr) {
  uchar ret;
  i2c_start();
  ret = i2c_put_u08(addr << 1);
  i2c_stop();
  return ret;
}

# if defined(DEBUG)
void i2c_scan(void) {
  _delay_ms(500);
  uchar i = 0;
  DEBUGF("Starting I2C scan\n");
  for(i=0;i<127;i++) {
    i2c_start();                  // do start transition
    if(i2c_put_u08(i << 1))       // send DEVICE address
      DEBUGF("I2C device at address 0x%x\n", i);

    i2c_stop();
  }
}
# endif

# if defined(GPIOE)
/* Read current port status on TCA9548A I2C Multiplexer */
uchar tcaread(void) {
 uchar ret;
 i2c_start();
 i2c_put_u08((ADDR_MUX<<1)|0x1);
 ret = i2c_get_u08(1);
 i2c_stop();
 DEBUGF("MUXR: %d\n", ret);
 return ret;
}

/* Switch to port 'i' on TCA9548A I2C Multiplexer */
void tcaselect(uchar i) {
  if (i>7||i<0) return;
  i2c_start();
  i2c_put_u08((ADDR_MUX<<1));
  DEBUGF("MUXW: %d\n", (1<<i));
  i2c_put_u08(1<<i);
  i2c_stop();
  _delay_ms(100);
  tcaread();
}

/* Data on each detected Cluster HAT */
struct chdta_data {
  unsigned char chat; // Was clusterhat found? 1=yes/0=no
  unsigned short major; // Major version
  unsigned short minor; // Minor version
  unsigned int addr; // i/o expander address
  unsigned char hubinvert; // 1=if hub is ENABLED by setting pin low
} chdta_status[CTRL_MAXPI/4];
# endif

#endif // I2C_HW

/* Emulated I2C device */
#define I2C_DEV_ADDR 		0x20
#define STATUSE_NEEDADDR 	1
#define STATUSE_GOTADDR 	2
#define CTRL_VERSION    	0x02    // Version of the struct
#define VERSION_MAJOR   	0x01    // Major version
#define VERSION_MINOR   	0x03    // Minor version

uchar emulated_status = STATUSE_NEEDADDR;
uchar emulated_address = 0;

/* I2C registers for our emulated device */
struct i2creg {
  /* 0x00 ro   uchar version;	*/ // Struct Schema Version 
  /* 0x01 ro */ uchar maxpi; 	// Max number of Pi Zero supported (2=pHAT, 16=half DA, 32=full DA, etc).
  /* 0x02 rw */ uchar order;	// Order number (used to sort multiple controllers)
  /* 0x03 rw */ uchar mode;	// Hole mode (Unused)
  /* 0x04 ro    uchar type;	*/ // 0x00=ClusterCTRL DA, 0x01=ClusterCTRL Triple, 0x02=ClusterCTRL pHAT, 0x03=ClusterCTRL A+6, 0x04=ClusterCTRL Stack
  /* 0x05 rw */ uchar data7;    // Optional data byte for command
  /* 0x06 rw */ uchar data6;    // Optional data byte for command
  /* 0x07 rw */ uchar data5;    // Optional data byte for command
  /* 0x08 rw */ uchar data4;    // Optional data byte for command
  /* 0x09 rw */ uchar data3; 	// Optional data byte for command
  /* 0x0a rw */ uchar data2;	// Optional data byte for command
  /* 0x0b rw */ uchar data1;    // Optional data byte for command
  /* 0x0c rw */ uchar data0;	// Optional data byte for command
  /* 0x0d rw */ uchar cmd;	// Command register
  /* 0x0e ro */ uchar status; 	// Status of last command (0x00=OK, 0x01 = Not supported, 0x02 No Pi Zero 0x03 unknown command, 0x04 should have reset)
  
#if defined(GPIOE)
  /* 0x20 ro */ uchar exists;	// Bits - HAT exists
  /* 0x21 ro */ // TODO uchar pos;	// Bits - POS (0=off,1=on)
  /* 0x22 ro */ // TODO uchar exp;	// Bits - Expander type (0=XRA1200P,1=PCA)
#endif

} reg;

/* I2C Commands (write to register 0x0d) */
#define I2C_CMD_INIT    	0x01 // Unused
#define I2C_CMD_RESCAN  	0x02 // Unused
#define I2C_CMD_ON		0x03 // Turn on Pi Zero in data0
#define I2C_CMD_OFF		0x04 // Turn off Pi Zero in data0
#define I2C_CMD_ALERT_ON	0x05 // Turn ALERT LED on (if type=hat data0=Hx / type=phat alert led on)
#define I2C_CMD_ALERT_OFF 	0x06 // Turn ALERT LED off (if type=hat data0=Hx / type=phat alert led off)
#define I2C_CMD_HUB_CYCLE	0x07 // Turn off USB hub wait data0*0.01 seconds and turn back on (if type=hat data1=Hx, if type=phat cycle hub)
#define I2C_CMD_HUB_ON		0x08 // Turn USB hub on (if type=hat data0=Hx / type=phat N/A)
#define I2C_CMD_HUB_OFF		0x09 // Turn USB hub off (if type=hat data0=Hx / type=phat N/A)
#define I2C_CMD_LED_EN		0x0a // Enable LED for Pi Zero data0 (if type=hat then data0 = HAT, if type=phat then data0 = Px)
#define I2C_CMD_LED_DIS 	0x0b // Disable LED for Pi Zero in data0 (if type=hat then data0 = HAT, if type=phat then data0 = Px)
#define I2C_CMD_PWR_ON		0x0c // Turn on PWR LED
#define I2C_CMD_PWR_OFF		0x0d // Turn off PWR LED
#define I2C_CMD_RESET		0x0e // Reset
#define I2C_CMD_GET_PSTATUS	0x0f // Get Pi Zero status (returned on data0-data3)
#define I2C_CMD_FAN		0x10 // Turn fan on (data0=1) or off (data0=0)
#define I2C_CMD_GETPATH		0x11 // Get USB path to Px (data0=x 0=controller) returned in data7-data0
#define I2C_CMD_USBBOOT_EN	0x12 // Turn on USBBOOT for Px (data0=x)
#define I2C_CMD_USBBOOT_DIS	0x13 // Turn off USBBOOT for Px (data0=x)
#define I2C_CMD_GET_USTATUS	0x14 // Get USBBOOT status for Px (data0=x)
#define I2C_CMD_SET_ORDER	0x15 // Set "order"
#define I2C_CMD_PING		0x90 // PING / NOP command
#define I2C_CMD_SAVE		0xF0 // Save current order/LED/usbboot/Pi Zero state for power on (ALERT is always off on startup)
#define I2C_CMD_SAVEDEFAULTS	0xF1 // Reset EEPROM to defaults
#define I2C_CMD_GETDATA		0xF2 // Return data - data0 = (0 - version firmware version minor=data0, major=data1)
#define I2C_CMD_SAVEORDER	0xF3 // Write only Order to EEPROM
#define I2C_CMD_SAVEUSBBOOT	0xF4 // Write only USBBOOT settings to EEPROM
#define I2C_CMD_SAVEPOS		0xF5 // Write only Power On State to EEPROM
#define I2C_CMD_SAVELED		0xF6 // Write only LED states to EEPROM

/* Read register 0x0e - 0x00 means idle (otherwise returns pending command) */

/* ------------------------------------------------------------------------- */

struct i2c_cmd {
  unsigned char type;
  unsigned char cmd;
  unsigned short flags;
  unsigned short addr;
  unsigned short len;  
};

#define STATUS_IDLE          0
#define STATUS_ADDRESS_ACK   1
#define STATUS_ADDRESS_NAK   2

static uchar status = STATUS_IDLE;

static uchar i2c_do_emulated(struct i2c_cmd *cmd) {
  if(reg.cmd != 0&&(cmd->flags&I2C_M_RD) == 0) { // If we're still running a command don't run another unless it's a read
   status = STATUS_ADDRESS_NAK;
  } else {
   status = STATUS_ADDRESS_ACK;
   emulated_status = STATUSE_NEEDADDR;
   expected = cmd->len;
   saved_cmd = cmd->cmd;
   saved_addr = cmd->addr;
  }
  /* more data to be expected? */
#ifndef USBTINY
  return(cmd->len?0xff:0x00);
#else
  return(((cmd->flags & I2C_M_RD) && cmd->len)?0xff:0x00);
#endif

}

static uchar i2c_do(struct i2c_cmd *cmd) {
/*
  uchar addr;

  // normal 7bit address 
  addr = ( cmd->addr << 1 );
  if (cmd->flags & I2C_M_RD )
    addr |= 1;

  if(cmd->cmd & CMD_I2C_BEGIN) 
    i2c_start();
  else 
    i2c_repstart();    

  // send DEVICE address
  if(!i2c_put_u08(addr)) {
    status = STATUS_ADDRESS_NAK;
    expected = 0;
    i2c_stop();
  } else {  
    status = STATUS_ADDRESS_ACK;
    expected = cmd->len;
    saved_cmd = cmd->cmd;
    saved_addr = cmd->addr;

    // check if transfer is already done (or failed) 
    if((cmd->cmd & CMD_I2C_END) && !expected) 
      i2c_stop();
  }
*/

#if defined(I2C_HW)
  // TODO: i2c_do
#else
  // TODO: i2c_do
  status = STATUS_ADDRESS_NAK;
#endif

  // more data to be expected?
#ifndef USBTINY
  return(cmd->len?0xff:0x00);
#else
  return(((cmd->flags & I2C_M_RD) && cmd->len)?0xff:0x00);
#endif
}

#ifndef USBTINY
uchar	usbFunctionSetup(uchar data[8]) {
  static uchar replyBuf[4];
  usbMsgPtr = replyBuf;
#else
extern	byte_t	usb_setup ( byte_t data[8] )
{
  byte_t *replyBuf = data;
#endif

  // DEBUGF("Setup: %d %d %d %d %d %d %d %d\n", data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);

  switch(data[1]) {

  case CMD_ECHO: // echo (for transfer reliability testing)
    DEBUGF("CMD_ECHO\n");
    replyBuf[0] = data[2];
    replyBuf[1] = data[3];
    return 2;
    break;

  case CMD_GET_FUNC:
    DEBUGF("CMD_GET_FUNC\n");
    memcpy_P(replyBuf, &func, sizeof(func));
    return sizeof(func);
    break;

  case CMD_SET_DELAY:
    DEBUGF("CMD_SET_DELAY\n");
    /* The delay function used delays 4 system ticks per cycle. */
    /* This gives 1/3us at 12Mhz per cycle. The delay function is */
    /* called twice per clock edge and thus four times per full cycle. */ 
    /* Thus it is called one time per edge with the full delay */ 
    /* value and one time with the half one. Resulting in */
    /* 2 * n * 1/3 + 2 * 1/2 n * 1/3 = n us. */
    clock_delay = *(unsigned short*)(data+2);
    if(!clock_delay) clock_delay = 1;
    clock_delay2 = clock_delay/2;
    if(!clock_delay2) clock_delay2 = 1;

    break;

  case CMD_I2C_IO:
  case CMD_I2C_IO + CMD_I2C_BEGIN:
  case CMD_I2C_IO                 + CMD_I2C_END:
  case CMD_I2C_IO + CMD_I2C_BEGIN + CMD_I2C_END:
    // these are only allowed as class transfers
    // DEBUGF("CMD_I2C addr=%d I2C_DEV_ADDR=%d\n", ((struct i2c_cmd*)data)->addr, I2C_DEV_ADDR);
    if(((struct i2c_cmd*)data)->addr == I2C_DEV_ADDR) return i2c_do_emulated((struct i2c_cmd*)data);

    return i2c_do((struct i2c_cmd*)data); // TODO: cleanup
    break;

  case CMD_GET_STATUS:
    DEBUGF("CMD_GET_STATUS=%d\n", status);
    replyBuf[0] = status;
    return 1;
    break;

  default:
    DEBUGF("CMD_DEFAULT\n");
    // must not happen ...
    break;
  }

  return 0;  // reply len
}


/*---------------------------------------------------------------------------*/
/* usbFunctionRead                                                           */
/*---------------------------------------------------------------------------*/

#ifndef USBTINY
uchar usbFunctionRead( uchar *data, uchar len )
#else
extern	byte_t	usb_in ( byte_t* data, byte_t len )
#endif
{
  uchar i;

  // DEBUGF("Read: len=%d\n", len);

  // DEBUGF("status=%d STATUS_ADDRESS_ACK=%d\n", status, STATUS_ADDRESS_ACK);

  if(status == STATUS_ADDRESS_ACK) {
    if(len > expected) {
      len = expected;
    }

    // DEBUGF("Saved_addr: %d I2C_DEV_ADDR=%d\n", saved_addr, I2C_DEV_ADDR);

    // Emulated device
    if(saved_addr==I2C_DEV_ADDR) {
      for(i=0;i<len;i++) {
        expected--;

        // DEBUGF("emulated_address=%d\n", emulated_address);

	if        (emulated_address==0x00) { // version
	 *data = CTRL_VERSION;
	} else if (emulated_address==0x01) { // maxpi
	 *data = CTRL_MAXPI;
	} else if (emulated_address==0x02) { // order
	 *data = reg.order;
	} else if (emulated_address==0x03) { // mode
	 *data = reg.mode;
        } else if (emulated_address==0x04) { // type
         *data = CTRL_TYPE;
	} else if (emulated_address==0x05) { // data7
	 *data = reg.data7;
        } else if (emulated_address==0x06) { // data6
         *data = reg.data6;
        } else if (emulated_address==0x07) { // data5
         *data = reg.data5;
        } else if (emulated_address==0x08) { // data4
         *data = reg.data4;
	} else if (emulated_address==0x09) { // data3
	 *data = reg.data3;
	} else if (emulated_address==0x0a) { // data2
	 *data = reg.data2;
	} else if (emulated_address==0x0b) { // data1
	 *data = reg.data1;
	} else if (emulated_address==0x0c) { // data0
	 *data = reg.data0;
	} else if (emulated_address==0x0d) { // cmd
	 *data = reg.cmd;
	} else if (emulated_address==0x0e) { // last status
	 *data = reg.status;
#if defined(GPIOE)
        } else if (emulated_address==0x20) { // exists
         *data = reg.exists;
#endif
	} else { // Undefined read 
         *data = 0xFF;
	}
	emulated_address++; // Increment register address
        data++;
      }
      return len;
    } // End Emulated device
  } else {
    memset(data, 0, len);
  }
  return len;
}

/*---------------------------------------------------------------------------*/
/* usbFunctionWrite                                                          */
/*---------------------------------------------------------------------------*/

#ifndef USBTINY
uchar usbFunctionWrite( uchar *data, uchar len )
#else
extern	void	usb_out ( byte_t* data, byte_t len )
#endif
{
  uchar left = len, i;

  // DEBUGF("Write: len=%d\n", len);

  if(status == STATUS_ADDRESS_ACK) {
    if(len > expected) {
      len = expected;
    }
    //
    // Emulated i2c device
    //
    if(saved_addr==I2C_DEV_ADDR) {

      if(emulated_status == STATUSE_NEEDADDR) {
	emulated_address = *data++;
        left = len-1;
      }

      for(i=0;i<left;i++) {
	// DEBUGF(" Byte:%d Data:%d Address: %d\n", i, *data, emulated_address);
       if        (emulated_address==0x00) { // version
	; // R/O
       } else if (emulated_address==0x01) { // maxpi
	; // R/O
       } else if (emulated_address==0x02) { // order
	if(*data==0) *data = 1;
	reg.order = *data;
       } else if (emulated_address==0x03) { // mode
	reg.mode = *data;
       } else if (emulated_address==0x04) { // type
	; // R/O
       } else if (emulated_address==0x05) { // data7
	 reg.data7 = *data; // data7
       } else if (emulated_address==0x06) { // data6
         reg.data6 = *data; // data6
       } else if (emulated_address==0x07) { // data5
         reg.data5 = *data; // data5
       } else if (emulated_address==0x08) { // data4
         reg.data4 = *data; // data4
       } else if (emulated_address==0x09) { // data3
	reg.data3 = *data; // data3
       } else if (emulated_address==0x0a) { // data2
        reg.data2 = *data; // data2
       } else if (emulated_address==0x0b) { // data1
        reg.data1 = *data; // data1
       } else if (emulated_address==0x0c) { // data0
        reg.data0 = *data; // data0
       } else if (emulated_address==0x0d) { // command
	reg.cmd = *data;
       } else if (emulated_address==0x0e) { // last status
	; // R/O
#if defined(GPIOE)
       } else if (emulated_address==0x20) { // exists
        ; // R/O
#endif
       } else {
	; // Do nothing
       }
       data++; // Increment pointer to data
       expected--;
       emulated_address++;
      }
#ifndef USBTINY
      return len;
#else
      return;
#endif
   }
   //
   // END Emulated device
   //
  } else {
    memset(data, 0, len);
  }

#ifndef USBTINY
  return len;
#endif
}

static void reset(void) {
	// Reduce watchdog timer interval
	wdt_enable(WDTO_15MS);
	// And sleep longer to force a reset via the watchdog
	_delay_ms(50);
}

#if defined (ADCEN)
static void adc_init() {
# if defined (ADCEN)
 ADMUX = (1<<REFS0); // Reference = AVCC (3v3)
 ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); // Enable with 128 prescaler (93kHz)
# endif
}
#else
#define adc_init()
#endif

static void save_led() {
	uchar i;

	DEBUGF("Save LED\n");

	// PWR LED
	if(state.enpwr==1) { // 0x01 EN PWR LED on boot
		eeprom_write_byte((uint8_t*)0x02, (uint8_t) 0x01);
	} else {
		eeprom_write_byte((uint8_t*)0x02, (uint8_t) 0x00);
	}

#if defined(LEDACTPORT)
	// ACT LED
	if(state.enact==1) { // 0x01 EN ACT LED on boot
		eeprom_write_byte((uint8_t*)0x03, (uint8_t) 0x01);
	} else {
		eeprom_write_byte((uint8_t*)0x03, (uint8_t) 0x00);
	}
#endif

	// Px LED
	for(i=0;i<CTRL_SIZE;i++) {
		eeprom_write_byte((uint8_t*)0x10+i, state.enpled[i]);
	}
}

static void save_order() {
	DEBUGF("Save order\n");
	eeprom_write_byte((uint8_t*)0x01,(uint8_t) reg.order);
}

static void save_pos() {
	uchar i;

	DEBUGF("Save POS\n");
	for(i=0;i<CTRL_SIZE;i++) {
		eeprom_write_byte((uint8_t*)0x30+i, state.enp[i]);
	}
}
#if defined(USBBOOT)
static void save_usbboot() {
	uchar i;

	DEBUGF("Save USBBOOT\n");
	for(i=0;i<CTRL_SIZE;i++) {
		eeprom_write_byte((uint8_t*)0x50+i, state.usbboot[i]);
	}
}
#endif

#if defined(GPIOE)

static uchar expander_read(uchar addr, uchar reg) {
 uchar ret;

 i2c_start();
 i2c_put_u08((addr<<1)|0x1);
 ret = i2c_get_u08(1);
 i2c_stop();
 DEBUGF("I/O Expander read addr: %d, reg:%d port:%d value:%d\n", addr, reg, tcaread(), ret);
 return ret;
}

static void expander_write(uchar addr, uchar reg, uchar value) {
 DEBUGF("I/O Expander write addr: %d, reg:%d port:%d value:%d\n", addr, reg, tcaread(), value);
 i2c_start();
 i2c_put_u08(addr<<1);
 i2c_put_u08(reg);
 i2c_put_u08(value);
 i2c_stop();
}

static uchar chdta_scan(uchar addr, uchar port) {
  uchar ret;
  uint32_t bytesRead = 0; // how many bytes we've read from the EEPROM
  uint16_t numatoms; // total atoms in EEPROM
  uint16_t atom; // Atom we're working on
  uint32_t eeplen; // total length in bytes of all eeprom data (including header)
  uint16_t tmp16; // 16 bit temp variable
  uint32_t dlen; // Length of the atom
  uint16_t atomtype; // Type of current atom
  uint32_t j; // Loop variable

  // TODO: check bytesRead doesn't go off end

  // Basic init - default to not found
  chdta_status[port].chat = 0;
  chdta_status[port].hubinvert = 0;
  chdta_status[port].addr = ADDR_PRI;

  // Set I2C MUX port
  tcaselect(port);
  wdt_reset();

  // Is there an HAT EEPROM?
  DEBUGF("Scanning for Cluster HAT on port %d?\n", port);
  if(!i2c_exists(ADDR_EEPROM)) {
   DEBUGF(" Not found\n");
   // If we didn't find an i2c device there is no HAT
   return 0;
  }

  // Initiate EEPROM READ
  i2c_start();
  i2c_put_u08((addr<<1));
  i2c_put_u08((0));
  i2c_put_u08((0));
  i2c_stop();

  // Check Signature 'R-Pi'

  // Read 0x00
  i2c_start();
  i2c_put_u08((addr<<1)|0x1);
  ret = i2c_get_u08(1);
  // DEBUGF("get 0x%x\n", ret);
  i2c_stop();
  if(ret!=0x52) { // 'R'
   return 0;
  }
  bytesRead++;

  // 0x01
  i2c_start();
  i2c_put_u08((addr<<1)|0x1);
  ret = i2c_get_u08(1);
  // DEBUGF("get 0x%x\n", ret);
  i2c_stop();
  if(ret!=0x2D) { // '-'
   return 0;
  }
  bytesRead++;

  // 0x02
  i2c_start();
  i2c_put_u08((addr<<1)|0x1);
  ret = i2c_get_u08(1);
  // DEBUGF("get 0x%x\n", ret);
  i2c_stop();
  if(ret!=0x50) { // 'P'
   return 0;
  }
  bytesRead++;

  // 0x03
  i2c_start();
  i2c_put_u08((addr<<1)|0x1);
  ret = i2c_get_u08(1);
  // DEBUGF("get 0x%x\n", ret);
  i2c_stop();
  if(ret!=0x69) { // 'i'
   return 0;
  }
  bytesRead++;

  // 0x04 // Header - version
  i2c_start();
  i2c_put_u08((addr<<1)|0x1);
  ret = i2c_get_u08(1);
  // DEBUGF("get 0x%x\n", ret);
  i2c_stop();
  if(ret!=0x01) { // Header version 0x01
   return 0;
  }
  bytesRead++;

  // 0x05 // Header reserved
  i2c_start();
  i2c_put_u08((addr<<1)|0x1);
  ret = i2c_get_u08(1);
  // DEBUGF("get 0x%x\n", ret);
  i2c_stop();
  bytesRead++;

  // 0x06 - numatoms (low)
  i2c_start();
  i2c_put_u08((addr<<1)|0x1);
  ret = i2c_get_u08(1);
  // DEBUGF("get 0x%x\n", ret);
  i2c_stop();
  bytesRead++;
  numatoms = ret;

  // 0x07 - numatoms (high)
  i2c_start();
  i2c_put_u08((addr<<1)|0x1);
  ret = i2c_get_u08(1);
  // DEBUGF("get 0x%x\n", ret);
  i2c_stop();
  bytesRead++;
  numatoms += ((uint16_t)ret<<8);

  // 0x08 - eeplen
  i2c_start();
  i2c_put_u08((addr<<1)|0x1);
  ret = i2c_get_u08(1);
  // DEBUGF("get 0x%x\n", ret);
  i2c_stop();
  bytesRead++;
  eeplen = ret;

  // 0x09
  i2c_start();
  i2c_put_u08((addr<<1)|0x1);
  ret = i2c_get_u08(1);
  // DEBUGF("get 0x%x\n", ret);
  i2c_stop();
  bytesRead++;
  eeplen += ((uint32_t)ret<<8);

  // 0x10
  i2c_start();
  i2c_put_u08((addr<<1)|0x1);
  ret = i2c_get_u08(1);
  // DEBUGF("get 0x%x\n", ret);
  i2c_stop();
  bytesRead++;
  eeplen += ((uint32_t)ret<<16);

  // 0x11 - eeplen (END)
  i2c_start();
  i2c_put_u08((addr<<1)|0x1);
  ret = i2c_get_u08(1);
  // DEBUGF("get 0x%x\n", ret);
  i2c_stop();
  bytesRead++;
  eeplen += ((uint32_t)ret<<24);
  // DEBUGF("eeplen %d\n", eeplen);
  // END OF Header

  // Loop through Atoms
  for(atom=0;atom<numatoms;atom++) {
   // DEBUGF("Start Atom\n");
   // Atom 0x00 - Atom type (low)
   i2c_start();
   i2c_put_u08((addr<<1)|0x1);
   ret = i2c_get_u08(1);
   // DEBUGF("get 0x%x\n", ret);
   i2c_stop();
   bytesRead++;
   atomtype = ret;
   // Atom 0x01 - Atom type (high)
   i2c_start();
   i2c_put_u08((addr<<1)|0x1);
   ret = i2c_get_u08(1);
   // DEBUGF("get 0x%x\n", ret);
   i2c_stop();
   bytesRead++;
   atomtype += ((uint16_t)ret<<8);

   // Atom 0x02 - atom count (low)
   i2c_start();
   i2c_put_u08((addr<<1)|0x1);
   ret = i2c_get_u08(1);
   // DEBUGF("get 0x%x\n", ret);
   i2c_stop();
   bytesRead++;
   tmp16 = ret;
   // Atom 0x03 - atom count (high)
   i2c_start();
   i2c_put_u08((addr<<1)|0x1);
   ret = i2c_get_u08(1);
   // DEBUGF("get 0x%x\n", ret);
   i2c_stop();
   bytesRead++;
   tmp16 += ((uint16_t)ret<<8);

   // Validate atom count = index
   if(atom!=tmp16) {
    DEBUGF("Incorrect Atom count %x %x \n", atom, tmp16);
    return 0;
   }

   // Atom 0x04 - dlen
   i2c_start();
   i2c_put_u08((addr<<1)|0x1);
   ret = i2c_get_u08(1);
   // DEBUGF("get 0x%x\n", ret);
   i2c_stop();
   bytesRead++;
   dlen = ret;
   // Atom 0x05 - dlen
   i2c_start();
   i2c_put_u08((addr<<1)|0x1);
   ret = i2c_get_u08(1);
   // DEBUGF("get 0x%x\n", ret);
   i2c_stop();
   bytesRead++;
   dlen += ((uint32_t)ret<<8);
   // Atom 0x06  - dlen
   i2c_start();
   i2c_put_u08((addr<<1)|0x1);
   ret = i2c_get_u08(1);
   // DEBUGF("get 0x%x\n", ret);
   i2c_stop();
   bytesRead++;
   dlen += ((uint32_t)ret<<16);
   // Atom 0x07 - dlen
   i2c_start();
   i2c_put_u08((addr<<1)|0x1);
   ret = i2c_get_u08(1);
   // DEBUGF("get 0x%x\n", ret);
   i2c_stop();
   bytesRead++;
   dlen += ((uint32_t)ret<<24);
   // DEBUGF("atomtype 0x%x\n", atomtype);
   if(atomtype==0x0001) {
    // Type = "vendor info"
    // Waste UUID bytes
    // DEBUGF("UUID\n");
    for(j=0;j<16;j++) {
     i2c_start();
     ret = i2c_put_u08((addr<<1)|0x1);
     ret = i2c_get_u08(1);
     i2c_stop();
    }
    // PID (low)
    i2c_start();
    i2c_put_u08((addr<<1)|0x1);
    ret = i2c_get_u08(1);
    // DEBUGF("get 0x%x\n", ret);
    i2c_stop();
    bytesRead++;
    tmp16 = ret;
    // PID (high)
    i2c_start();
    i2c_put_u08((addr<<1)|0x1);
    ret = i2c_get_u08(1);
    // DEBUGF("get 0x%x\n", ret);
    i2c_stop();
    bytesRead++;
    tmp16 += ((uint16_t)ret<<8);

    // Validate PID 0x0004
    if(tmp16!=0x0004) {
     DEBUGF("Invalid Product ID 0x%x\n", tmp16);
     return 0;
    }
    // Product Version (low)
    i2c_start();
    i2c_put_u08((addr<<1)|0x1);
    ret = i2c_get_u08(1);
    // DEBUGF("get 0x%x\n", ret);
    i2c_stop();
    bytesRead++;
    tmp16 = ret;
    // Product Version (high)
    i2c_start();
    i2c_put_u08((addr<<1)|0x1);
    ret = i2c_get_u08(1);
    // DEBUGF("get 0x%x\n", ret);
    i2c_stop();
    bytesRead++;
    if(ret!=0x00) { // High byte should be 0x00 for ClusterHAT
    // DEBUGF("Invalid Product Version\n");
     return 0;
    }

    // Set Cluster HAT major/minor versions
    chdta_status[port].minor = (tmp16&0xF); // Low nibble
    chdta_status[port].major = (tmp16>>4);  // High nibble

    // If v1.x then we need to use onboard I/O expander
    if(chdta_status[port].major==0x1) { // v1.x
     chdta_status[port].addr = ADDR_SEC;
    } else if (chdta_status[port].major==0x2) { // v2.x
     if(chdta_status[port].minor>0) { // starting 2.1 HUB is active low
      chdta_status[port].hubinvert = 1;
     }
    }

    // Can we talk to the I/O expander?
    if(!i2c_exists(chdta_status[port].addr)) {
     return 0;
    }

    // Don't need to check Manufacturer / Product string

    // Turn off all Pi Zeros / enable LED & HUB
    expander_write(chdta_status[port].addr, XRA1200_OUT, 0x00|(1<<X_LED)|(!chdta_status[port].hubinvert<<X_HUB));

    // Set all pins as outputs
    expander_write(chdta_status[port].addr, XRA1200_DIR, 0x00);

    DEBUGF("Port: %d Cluster HAT v%d.%d\n", port, chdta_status[port].major, chdta_status[port].minor);
    
    chdta_status[port].chat = 1; // Cluster HAT found

    /* DONE */
    return 1;
   } else {
    // Not "vendor info" Atom - waste bytes
    for(j=0;j<dlen;j++) {
     i2c_start();
     i2c_put_u08((addr<<1)|0x1);
     i2c_get_u08(1);
     i2c_stop();
    }
   } // END if atomtype=0x0001
  } // END Loop Atoms

  // If we've dropped out of the loop we didn't find a Cluster HAT
  return 0;
}

/* Init Cluster HAT DTA 
 * returns bitmask of ClusterHAT found
 */
uchar chdta_init() {
 uchar i;
 uchar ret = reg.exists;
 uchar tca = tcaread();

 ret = 0;
 DEBUGF("HAT Init\n");

 for(i=0;i<8;i++) {
  if(chdta_scan(ADDR_EEPROM, i)) ret |= (1<<i);
  wdt_reset();
  usbPoll();
 }
 reg.exists = ret;
 DEBUGF("HAT Init done\n");

 tcaselect(tca);

 return ret;
}

#endif

/* ------------------------------------------------------------------------- */

int	main(void) {
  wdt_enable(WDTO_8S);
  uint8_t tmpu8;
  uchar i;

#if defined(GPIOE)
 // No HAT exists on power up
 reg.exists=0; // TODO reg.pos=reg.exp=0;

 // Init Software I2C
 i2c_init();
#endif

#if DEBUG_LEVEL > 0
  /* let debug routines init the uart if they want to */
  odDebugInit();
#else
#if defined(UARTDEBUG)
  /* quick'n dirty uart init */
  UCSR0B |= _BV(TXEN0);
  UBRR0L = F_CPU / (19200 * 16L) - 1;
#endif
#endif

#if defined(UARTDEBUG)
  stdout = &mystdout;
#endif

#if defined(FANSTATUS)
 fanstatus=0;
#endif

DEBUGF("INIT\n");

/* Set ports as input/output */
#if defined(DDRBINIT)
  DDRB |= DDRBINIT;
#endif
#if defined(DDRCINIT)
  DDRC |= DDRCINIT;
#endif
#if defined(DDRDINIT)
  DDRD |= DDRDINIT;
#endif

/* Always Enable the USB hub */
#if defined(HUBENPORT) && defined(HUBENPIN)
# if defined(HUBENHIGH)
  HUBENPORT |= (1 << HUBENPIN); // HUB EN (active high)
# else
  HUBENPORT &= ~(1 << HUBENPIN); // HUB EN (active low)
# endif
#endif

/* Always enable the MUX */
#if defined(MUXRESETPORT) && defined(MUXRESETPIN)
# if defined(MUXRESETHIGH)
  MUXRESETPORT |= (1<<MUXRESETPIN); // Enable MUX (active high)
# else
  MUXRESETPORT &= ~(1<<MUXRESETPIN); // Enable MUX (active low)
# endif
#endif

/* Always enable the Regulator */
#if defined(REG1PORT) && defined(REG1PIN)
# if defined(REG1HIGH)
  REG1PORT |= (1<<REG1PIN); // Enable Regulator 1 (active high)
# else
  REG1PORT &= ~(1<<REG1PIN); // Enable Regulator 1 (active low)
# endif
#endif

// Set defaults for non-saved options
#if defined(LEDALERTPORT) && defined(LEDALERTPIN)
  LEDALERTPORT &= ~(1<<LEDALERTPIN);
#endif


/* EEPROM locations for power on defaults

 0x01   Order (1-255) used by clusterctl script to work out Px number with multiple devices
 0x02   PWR LED enable (1=on)
 0x03   ACT LED enable (1=on)
 0x04-0x0F   ...
 0x10+y Px LED enable (bitfield 1=on) 
 0x30+y PWR Px enable (bitfield 1=on)
 0x50+y USBBOOT enable (bitfield 1=on)
 0x70   ...

*/

  // Get defaults from EEPROM

  // Get Order
  reg.order = eeprom_read_byte((uint8_t*)0x01);

  // Get the state of the PWR LED
  tmpu8 = eeprom_read_byte((uint8_t*)0x02);
  if(tmpu8==0x01) {
   state.enpwr = 1; // Enable
   LEDPWRPORT |= (1<<LEDPWRPIN);
  } else {
   state.enpwr = 0; // Disable
   LEDPWRPORT &= ~(1<<LEDPWRPIN);
  }

#if defined(LEDACTPORT) && defined(LEDACTPIN)
  // Get the state of the ACT LED
  tmpu8 = eeprom_read_byte((uint8_t*)0x03);
  if(tmpu8==0x01) {
   state.enact = 1; // Enable
   LEDACTPORT &= ~(1<<LEDACTPIN); // Turn off
  } else {
   state.enact = 0; // Disable
   LEDACTPORT &= ~(1<<LEDACTPIN); // Turn off
  }
#endif

  // Read Pxi, PxLED and PxUSBBOOT default states from EEPROM
  for(i=0;i<CTRL_SIZE;i++) {
   state.enpled[i] = eeprom_read_byte((uint8_t*)0x10+i);
   state.enp[i] = eeprom_read_byte((uint8_t*)0x30+i);
#if defined(USBBOOT)
   state.usbboot[i] = eeprom_read_byte((uint8_t*)0x50+i);
#endif

#if defined(GPIOD)
# if defined(ENP1PORT) && defined(ENP1PIN)
   if((state.enp[0]>>0)&0x01) {
    // Turn on P1
    ENP1PORT |= (1<<ENP1PIN);
    // Is P1LED is enabled
#  if defined(LEDP1PORT) && defined(LEDP1PIN)
    if((state.enpled[0]>>0)&0x01) {
     LEDP1PORT |= (1<<LEDP1PIN);
    }
#  endif
   }
# endif // P1
# if defined(ENP2PORT) && defined(ENP2PIN)
   if((state.enp[0]>>1)&0x01) {
#  if defined(AUTOONDELAY)
   _delay_ms(AUTOONDELAY);
#  endif
    // Turn on P2
    ENP2PORT |= (1<<ENP2PIN);
    // Is P2LED is enabled
#  if defined(LEDP2PORT) && defined(LEDP2PIN)
    if((state.enpled[0]>>1)&0x01) {
     LEDP2PORT |= (1<<LEDP2PIN);
    }
#  endif
   }
# endif // P2
# if defined(USBBOOTP1PORT) && defined(USBBOOTP1PIN)
   if( (state.usbboot[0]>>0)&0x1 ) {
    USBBOOTP1PORT |= (1<<USBBOOTP1PIN);
   }
# endif
# if defined(USBBOOTP2PORT) && defined(USBBOOTP2PIN)
   if( (state.usbboot[0]>>1)&0x1 ) {
    USBBOOTP2PORT |= (1<<USBBOOTP2PIN);
   }
# endif
#elif defined(GPIOA)
   for(tmpu8=0;tmpu8<8;tmpu8++) { // Loop through each bit
    if( ((i*8)+tmpu8)<CTRL_MAXPI ) {
     if( ((state.enp[i]>>tmpu8)&0x1) ) { // Px is enabled
# if defined(AUTOONDELAY)
     if (tmpu8>0) _delay_ms(AUTOONDELAY);
# endif
      (*p[((i*8)+tmpu8)][0]) |= (1 << (int)p[((i*8)+tmpu8)][1]);
      // Is LEDPx enabled?
# if !defined(NOPLED)
      if( ((state.enpled[i]>>tmpu8)&0x1) ) { // PxLED is enabled
       (*pled[((i*8)+tmpu8)][0]) |= (1<< (int)pled[((i*8)+tmpu8)][1]);
      }
# endif
     }
# if defined(USBBOOT)
     if( ((state.usbboot[i]>>tmpu8)&0x1) ) { // PxUSBBOOT is enabled
      (*usbboot[((i*8)+tmpu8)][0]) |= (1 << (int)usbboot[((i*8)+tmpu8)][1]);
     }
# endif
    }
   }
#endif
  }

#if defined(GPIOE)
  chdta_init();
#endif

  /* clear usb ports */
  USB_CFG_IOPORT   &= (uchar)~((1<<USB_CFG_DMINUS_BIT)|(1<<USB_CFG_DPLUS_BIT));

  /* make usb data lines outputs */
  USBDDR    |= ((1<<USB_CFG_DMINUS_BIT)|(1<<USB_CFG_DPLUS_BIT));

  /* USB Reset by device only required on Watchdog Reset */
  _delay_loop_2(40000);   // 10ms

  /* make usb data lines inputs */
  USBDDR &= ~((1<<USB_CFG_DMINUS_BIT)|(1<<USB_CFG_DPLUS_BIT));

  usbInit();

  sei();

  for(;;) {	/* main event loop */
    wdt_reset(); 
    usbPoll();

    if(reg.cmd != 0) {
     DEBUGF("## COMMAND %d\n", reg.cmd);
     DEBUGF("## DATA0 %d %d %d %d\n", reg.data0, reg.data1, reg.data2, reg.data3);
     if(reg.cmd==I2C_CMD_ON) { // Turn Pi Zero on
      DEBUGF("##ON %d\n", reg.data0);
      if(reg.data0>0&&reg.data0<=CTRL_MAXPI) {
       DEBUGF("P%d ON\n", reg.data0);
#if defined(GPIOD)
# if defined(ENP1PORT) && defined(ENP1PIN)
       if(reg.data0==1) {
        ENP1PORT |= (1<<ENP1PIN);
        state.enp[0] |= 1<<0;
        // If P1LED is enabled turn it on
#  if defined(LEDP1PORT) && defined(LEDP1PIN)
        if((state.enpled[0]>>0)&0x01) {
         LEDP1PORT |= (1<<LEDP1PIN);
        }
#  endif
       }
# endif
# if defined(ENP2PORT) && defined(ENP2PIN)
       if(reg.data0==2) {
        ENP2PORT |= (1<<ENP2PIN);
        state.enp[0] |= 1<<1;
        // If P2LED is enabled turn it on
#  if defined(LEDP2PORT) && defined(LEDP2PIN)
        if((state.enpled[0]>>1)&0x01) {
         LEDP2PORT |= (1<<LEDP2PIN);
        }
#  endif
       }
# endif
#elif defined(GPIOA)
       // Turn on Px
       (*p[reg.data0-1][0]) |= 1<< (int)p[reg.data0-1][1];
       // Update state
       state.enp[ P2BYTE(reg.data0-1) ] |= 1<< P2BIT(reg.data0-1);
       // If PxLED is enabled turn it on
# if !defined(NOPLED)
       if( state.enpled[ P2BYTE(reg.data0-1) ]&(1<<P2BIT(reg.data0-1)) ) {
        (*pled[reg.data0-1][0]) |= 1<< (int)pled[reg.data0-1][1];
       }
# endif
#elif defined(GPIOE)
       reg.data0--;
       if(chdta_status[reg.data0/4].chat==1) {
        // Switch MUX
        tcaselect(reg.data0/4);
        // Turn Px on
        expander_write(chdta_status[reg.data0/4].addr, XRA1200_OUT, expander_read(chdta_status[reg.data0/4].addr, XRA1200_IN)|(1<<(reg.data0%4)));
       }
#endif
      }
      reg.cmd=0; // DONE
      reg.status=0x0; // No error
     } else if (reg.cmd==I2C_CMD_OFF) { // Turn Pi Zero off
      DEBUGF("##OFF %d\n", reg.data0);
      if(reg.data0>0&&reg.data0<=CTRL_MAXPI) {
       DEBUGF("P%d OFF\n", reg.data0);
#if defined(GPIOD)
# if defined(ENP1PORT) && defined(ENP1PIN)
       if(reg.data0==1) {
        // Turn P1 power off
        ENP1PORT &= ~(1<<ENP1PIN);
        // Update state
        state.enp[0] &= ~(1<<0);
        // Turn LED off
#  if defined(LEDP1PORT) && defined(LEDP1PIN)
        LEDP1PORT &= ~(1<<LEDP1PIN);
#  endif
       }
# endif // P1
# if defined(ENP2PORT) && defined(ENP2PIN)
       if(reg.data0==2) {
        // Turn P2 power off
        ENP2PORT &= ~(1<<ENP2PIN);
        // Update state
        state.enp[0] &= ~(1<<1);
        // Turn LED off
#  if defined(LEDP2PORT) && defined(LEDP2PIN)
        LEDP2PORT &= ~(1<<LEDP2PIN);
#  endif
       }
# endif // P2
#elif defined(GPIOA)
       // Turn power off
       (*p[reg.data0-1][0]) &= ~(1 << (int)p[reg.data0-1][1]);
       // Update state
       state.enp[ P2BYTE(reg.data0-1) ] &= ~(1<< P2BIT(reg.data0-1));
       // Turn PxLED off
# if !defined(NOPLED)
       (*pled[reg.data0-1][0]) &= ~(1 << (int)pled[reg.data0-1][1]);
# endif
#elif defined(GPIOE)
       reg.data0--;
       if(chdta_status[reg.data0/4].chat==1) {
        // Switch MUX
        tcaselect(reg.data0/4);
        // Turn Px off
        expander_write(chdta_status[reg.data0/4].addr, XRA1200_OUT, expander_read(chdta_status[reg.data0/4].addr, XRA1200_IN)&~(1<<(reg.data0%4)));
       }
#endif
      }
      reg.cmd=0; // DONE
      reg.status=0x0; // No error
#if defined(USBBOOT)
     } else if(reg.cmd==I2C_CMD_USBBOOT_EN) { // Turn USBBOOT on
      DEBUGF("##USBBOOT ON %d\n", reg.data0);
      if(reg.data0>0&&reg.data0<=CTRL_MAXPI) {
       DEBUGF("P%d ON\n", reg.data0);
# if defined(GPIOD)
#  if defined(USBBOOTP1PORT) && defined(USBBOOTP1PIN)
       if(reg.data0==1) {
        USBBOOTP1PORT |= (1<<USBBOOTP1PIN);
        state.usbboot[0] |= (1<<0);
       }
#  endif
#  if defined(USBBOOTP2PORT) && defined(USBBOOTP2PIN)
       if(reg.data0==2) {
        USBBOOTP2PORT |= (1<<USBBOOTP2PIN);
        state.usbboot[0] |= (1<<1);
       }
#  endif
# elif defined(GPIOA)
       (*usbboot[reg.data0-1][0]) |= 1<< (int)usbboot[reg.data0-1][1];
# endif
       state.usbboot[ (P2BYTE(reg.data0-1)) ] |= (1<<P2BIT(reg.data0-1));
      }
      reg.cmd=0; // DONE
      reg.status=0x0; // No error
     } else if(reg.cmd==I2C_CMD_USBBOOT_DIS) { // Turn USBBOOT off
      DEBUGF("##USBBOOT OFF %d\n", reg.data0);
      if(reg.data0>0&&reg.data0<=CTRL_MAXPI) {
       DEBUGF("P%d OFF\n", reg.data0);
# if defined(GPIOD)
#  if defined(USBBOOTP1PORT) && defined(USBBOOTP1PIN)
       if(reg.data0==1) {
        USBBOOTP1PORT &= ~(1<<USBBOOTP1PIN);
        state.usbboot[0] &= ~(1<<0);
       }
#  endif
#  if defined(USBBOOTP2PORT) && defined(USBBOOTP2PIN)
      if(reg.data0==2) {
       USBBOOTP2PORT &= ~(1<<USBBOOTP2PIN);
        state.usbboot[0] &= ~(1<<1);
      }
#  endif
# elif defined(GPIOA)
       (*usbboot[reg.data0-1][0]) &= ~(1 << (int)usbboot[reg.data0-1][1]);
# endif
       state.usbboot[ (P2BYTE(reg.data0-1)) ] &= ~(1<<P2BIT(reg.data0-1));
      }
      reg.cmd=0; // DONE
      reg.status=0x0; // No error
#endif
     } else if (reg.cmd==I2C_CMD_LED_EN) { // Enable Px LED
      if(reg.data0==0) { // All PxLED+PWR
       // Enable PWR LED
       state.enpwr=1;
       LEDPWRPORT |= (1<<LEDPWRPIN);
       // Enable PxLED and turn on if Px is on
#if defined(GPIOD)
# if defined(ENP1PORT) && defined(ENP1PIN)
        ENP1PORT |= (1<<ENP1PIN);
#  if defined(LEDP1PORT) && defined(LEDP1PIN)
        LEDP1PORT |= (1<<LEDP1PIN);
#  endif
# endif
# if defined(ENP2PORT) && defined(ENP2PIN)
        ENP2PORT |= (1<<ENP2PIN);
#  if defined(LEDP2PORT) && defined(LEDP2PIN)
        LEDP2PORT |= (1<<LEDP2PIN);
#  endif
# endif
        state.enp[0] = 0xFF; // Just set all bits
        state.enpled[0] = 0xFF;
#elif defined(GPIOA)
       for(tmpu8=1;tmpu8<=CTRL_MAXPI;tmpu8++) {
        state.enpled[ (P2BYTE(tmpu8-1)) ] |= (1<<P2BIT(tmpu8-1));
# if !defined(NOPLED)
        if( state.enp[ (P2BYTE(tmpu8-1)) ]&(1<<P2BIT(tmpu8-1)) ) {
         (*pled[tmpu8-1][0]) |= 1<< (int)pled[tmpu8-1][1];
        }
# endif
       }
#endif
      } else if(reg.data0>0&&reg.data0<=CTRL_MAXPI) {
       DEBUGF("Enable LED for P%d\n", reg.data0);
       // If Px is on turn the LED on
#if defined(GPIOD)
# if defined(LEDP1PORT) && defined(LEDP1PIN)
        if( (state.enp[0]>>0)&0x1 && reg.data0==1) {
         LEDP1PORT |= (1<<LEDP1PIN);
         state.enpled[0] |= (1<<0);
        }
# endif
# if defined(LEDP2PORT) && defined(LEDP2PIN)
        if( (state.enp[0]>>1)&0x1 && reg.data0==2) {
         LEDP2PORT |= (1<<LEDP2PIN);
         state.enpled[0] |= (1<<1);
        }
# endif
#elif defined(GPIOA)
       state.enpled[ (P2BYTE(reg.data0-1)) ] |= (1<<P2BIT(reg.data0-1));
# if !defined(NOPLED)
       if( state.enp[ (P2BYTE(reg.data0-1)) ]&(1<<P2BIT(reg.data0-1)) ) {
        (*pled[reg.data0-1][0]) |= 1<< (int)pled[reg.data0-1][1];
       }
# endif
#endif
      }
      reg.cmd=0; // DONE
      reg.status=0x0; // No error
     } else if (reg.cmd==I2C_CMD_LED_DIS) { // Disable Px LED
        if(reg.data0==0) { // All PxLED+PWR
         // Disable PWR LED
         state.enpwr=0;
         LEDPWRPORT &= ~(1<<LEDPWRPIN);
         // Disable PxLED
#if defined(GPIOD)
# if defined(LEDP1PORT) && defined(LEDP1PIN)
         LEDP1PORT &= ~(1<<LEDP1PIN);
# endif
# if defined(LEDP2PORT) && defined(LEDP2PIN)
         LEDP2PORT &= ~(1<<LEDP2PIN);
# endif
         state.enpled[0] = 0x00;
#elif defined(GPIOA)
         for(tmpu8=1;tmpu8<=CTRL_MAXPI;tmpu8++) {
          state.enpled[ (P2BYTE(tmpu8-1)) ] &= ~(1<<P2BIT(tmpu8-1));
# if !defined(NOPLED)
          (*pled[tmpu8-1][0]) &= ~(1<< (int)pled[tmpu8-1][1]);
# endif
         }
#endif
        } else if(reg.data0>0&&reg.data0<=CTRL_MAXPI) {
         DEBUGF("Disable LED for P%d\n", reg.data0);
         // Turn off Px LED
#if defined(GPIOD)
# if defined(LEDP1PORT) && defined(LEDP1PIN)
        if(reg.data0==1) {
         LEDP1PORT &= ~(1<<LEDP1PIN);
         state.enpled[0] &= ~(1<<0);
        }
# endif
# if defined(LEDP2PORT) && defined(LEDP2PIN)
        if(reg.data0==2) {
         LEDP2PORT &= ~(1<<LEDP2PIN);
         state.enpled[0] &= ~(1<<1);
        }
# endif
#elif defined(GPIOA)
         state.enpled[ (P2BYTE(reg.data0-1)) ] &= ~(1<<P2BIT(reg.data0-1));
# if !defined(NOPLED)
         (*pled[reg.data0-1][0]) &= ~(1<< (int)pled[reg.data0-1][1]);
# endif
#endif
        }
        reg.cmd=0; // DONE
        reg.status=0x0; // No error
     } else if (reg.cmd==I2C_CMD_ALERT_ON) { // Alert on
#if defined(LEDALERTPORT) && defined(LEDALERTPIN)
	DEBUGF("Alert on\n");
	LEDALERTPORT |= (1<<LEDALERTPIN);
#endif
	reg.cmd=0; // DONE
	reg.status=0x0; // No error
     } else if (reg.cmd==I2C_CMD_ALERT_OFF) { // Alert off
	DEBUGF("Alert off\n");
#if defined(LEDALERTPORT) && defined(LEDALERTPIN)
	LEDALERTPORT &= ~(1<<LEDALERTPIN);
#endif
	reg.cmd=0; // DONE
	reg.status=0x0; // No error
     } else if (reg.cmd==I2C_CMD_SAVE) { // Save state
	DEBUGF("Save all states\n");
	save_order();

	save_led();

	save_pos();

#if defined(USBBOOT)
	save_usbboot();
#endif

	reg.cmd=0; // DONE
	reg.status=0x0; // OK
     } else if (reg.cmd==I2C_CMD_SAVEUSBBOOT) { // Save state USBBOOT
#if defined(USBBOOT)
	save_usbboot();
#endif
	reg.cmd=0; // DONE
	reg.status=0x0; // OK
     } else if (reg.cmd==I2C_CMD_SAVEORDER) { // Save state ORDER
	save_order();
	reg.cmd=0; // DONE
	reg.status=0x0; // OK
     } else if (reg.cmd==I2C_CMD_SAVEPOS) { // Save state Pi Power On State
	save_pos();
	reg.cmd=0; // DONE
	reg.status=0x0; // OK
     } else if (reg.cmd==I2C_CMD_SAVELED) { // Save state LED
	save_led();
	reg.cmd=0; // DONE
	reg.status=0x0; // OK
     } else if (reg.cmd==I2C_CMD_HUB_CYCLE) { // Cycle USB hub
	DEBUGF("HUB cycle\n");
	if(reg.data0<10)  reg.data0=10;  // Min off time is 100ms (0.01*10)
#if defined(HUBENPIN)
# if defined(HUBENHIGH) // Turn USB hub off
	HUBENPORT &= ~(1<<HUBENPIN);
# else
	HUBENPORT |= (1<<HUBENPIN);
# endif
#endif
	for(i=0;i<reg.data0;i++) {
		_delay_loop_2(40000);   // 10ms
	}
#if defined(HUBENPIN)
# if defined(HUBENHIGH) // Turn USB hub back on
	HUBENPORT |= (1<<HUBENPIN);
# else
	HUBENPORT &= ~(1<<HUBENPIN);
# endif
#endif
	reg.cmd=0; // DONE
     } else if (reg.cmd==I2C_CMD_SAVEDEFAULTS) { // Reset EEPROM to defaults
	DEBUGF("Reset EEPROM\n");
	eeprom_write_byte((uint8_t*)0x01, 20); // Order
	eeprom_write_byte((uint8_t*)0x02, 0x01); // 0x01 EN PWR LED on boot
	eeprom_write_byte((uint8_t*)0x03, 0x01); // 0x03 EN ACT LED

	for(i=0;i<CTRL_SIZE;i++) {
		DEBUGF("Reset Byte %d\n", i);
                eeprom_write_byte((uint8_t*)0x10+i, 0xFF); // PxLED Enabled
                eeprom_write_byte((uint8_t*)0x30+i, 0x00); // Px OFF
#if defined(USBBOOT)
		eeprom_write_byte((uint8_t*)0x50+i, 0xFF); // USBBOOT Enabled
#endif
        }

        reg.cmd=0; // DONE
	reg.status=0x00; // No error
     } else if (reg.cmd==I2C_CMD_PWR_ON) { // Turn on PWR LED
	DEBUGF("Enable PWR LED\n");
	LEDPWRPORT|= (1<<LEDPWRPIN);
	state.enpwr=1;
	reg.cmd=0;  // DONE
	reg.status=0x00; // No error
     } else if (reg.cmd==I2C_CMD_PWR_OFF) { // Turn off PWR LED
	DEBUGF("Disable PWR LED\n");
	LEDPWRPORT &= ~(1<<LEDPWRPIN);
	state.enpwr=0;
	reg.cmd=0; // DONE
	reg.status=0x00; // No error
     } else if (reg.cmd==I2C_CMD_RESET) { // Reset
	DEBUGF("RESET\n");
	reset();
	// We should never get here
	reg.cmd=0; // DONE
	reg.status=0x00; // No error
     } else if (reg.cmd==I2C_CMD_GET_PSTATUS) { // Get power status
	DEBUGF("\n\nGet power status %d\n", reg.data0);
        if(reg.data0>0&&reg.data0<=CTRL_MAXPI) {
#if defined(GPIOD)
# if defined(LEDP1PORT) && defined(LEDP1PIN)
         if(reg.data0==1) {
          reg.data0=((state.enp[0]>>0)&0x01);
         }
# endif
# if defined(LEDP2PORT) && defined(LEDP2PIN)
         if(reg.data0==2) {
          reg.data0=((state.enp[0]>>1)&0x01);
         }
# endif
#elif defined(GPIOA)
	 reg.data0 = (((*p[reg.data0-1][0]) >> (int)p[reg.data0-1][1])&0x1);
#elif defined(GPIOE)
         reg.data0--;
         if(chdta_status[reg.data0/4].chat==1) {
          // Switch MUX
          tcaselect(reg.data0/4);
          // Read status and mask
          reg.data0 = (expander_read(chdta_status[reg.data0/4].addr, XRA1200_IN)&(1<<(reg.data0%4)));
          reg.data0 = reg.data0>0?1:0;
          DEBUGF("PSTATUS: %d\n", reg.data0);
         } else {
          reg.data0=0xFF;
         }
#endif
	 reg.cmd=0; // DONE
	 reg.status=0x00; // No error
	} else {
	 reg.data0=0xFF;
	 reg.cmd=0; // DONE
	 reg.status=0x02; // No error
	}
	DEBUGF("data0: %d\n", reg.data0);
     } else if (reg.cmd==I2C_CMD_GET_USTATUS) { // Get USBBOOT status
#if defined(USBBOOT)
      DEBUGF("\n\nGet USBBOOT status %d\n", reg.data0);
      if(reg.data0>0&&reg.data0<=CTRL_MAXPI) {
# if defined(GPIOD)
#  if defined(USBBOOTP1PORT) && defined(USBBOOTP1PIN)
      if(reg.data0==1) {
       reg.data0=((state.usbboot[0]>>0)&0x01);
      }
#  endif
#  if defined(USBBOOTP2PORT) && defined(USBBOOTP2PIN)
      if(reg.data0==2) {
       reg.data0=((state.usbboot[0]>>1)&0x01);
      }
#  endif
# elif defined(GPIOA)
       reg.data0 = (((*usbboot[reg.data0-1][0]) >> (int)usbboot[reg.data0-1][1])&0x1);
# endif
       reg.cmd=0; // DONE
       reg.status=0x00; // No error
      } else {
       reg.data0=0xFF;
      }
#else
      reg.data0=0xFF;
#endif
      reg.cmd=0; // DONE
      reg.status=0x00; // No error
#if ( defined (FANENPORT) && defined (FANENPIN) ) || defined(FANSTATUS)
     } else if (reg.cmd==I2C_CMD_FAN) { // Turn fan on/off
# if defined (FANSTATUS)
        fanstatus=reg.data0;
# endif
# if  defined (FANENPORT) && defined (FANENPIN)
	if(reg.data0==1) { // Turn on
		DEBUGF("FAN ON %d\n", FANENPORT);
		FANENPORT |= (1<<FANENPIN);
		DEBUGF("FAN ON done %d\n", FANENPORT);
	} else { // Turn off
		DEBUGF("FAN OFF %d\n", FANENPORT);
		FANENPORT &= ~(1<<FANENPIN);
		DEBUGF("FAN OFF done %d\n", FANENPORT);
	}
# endif
	reg.cmd=0; // DONE
	reg.status=0x00; // No error
#endif
     } else if (reg.cmd==I2C_CMD_GETDATA) {
      reg.status=0x01; // (fallback) Unsupported
      DEBUGF("GETDATA\n");
      if(reg.data0==0) { // Get version
       DEBUGF("- Version\n");
       reg.data0=VERSION_MINOR;
       reg.data1=VERSION_MAJOR;
       reg.data2=0xFF;
       reg.data3=0xFF;
       reg.data4=0xFF;
       reg.data5=0xFF;
       reg.data6=0xFF;
       reg.data7=0xFF;
       reg.status=0x00; // No error
      } else if(reg.data0==1) { // Get number of ADC
       DEBUGF("- Number of ADC\n");
       reg.data0 = 0;
#if defined(ADC1)
       reg.data0++;
# if defined(ADC2)
       reg.data0++;
# endif
#endif
       reg.status=0x00; // No error
      } else if(reg.data0==2) { // Read ADC #reg.data1
       DEBUGF("- Read ADC %d\n", reg.data1);
#if defined(ADC1)
       if(reg.data1==1) {
        adc_init();
        tmpu8 = ADC1;
        tmpu8 &= 0b00000111;
        ADMUX = (ADMUX & 0xF8)|tmpu8;
        ADCSRA |= (1<<ADSC);
        while(ADCSRA & (1<<ADSC));
        reg.data0 = ADC1TYPE;
	reg.data1 = ADCL; // ADC Low byte
	reg.data2 = ADCH; // ADC High byte
	DEBUGF("L: %d\n", reg.data1);
	DEBUGF("H: %d\n", reg.data2);
	DEBUGF("ADC1 %d mV\n", (int)((double)( (reg.data2<<8)+reg.data1 )*6.4453125) );
        reg.status=0x00; // No error
       }
#endif
#if defined(ADC2)
       if(reg.data1==2) {
        adc_init();
        tmpu8 = ADC2;
        tmpu8 &= 0b00000111;
        ADMUX = (ADMUX & 0xF8)|tmpu8;
        ADCSRA |= (1<<ADSC);
        while(ADCSRA & (1<<ADSC));
        reg.data0 = ADC2TYPE;
        reg.data1 = ADCL; // ADC Low byte
        reg.data2 = ADCH; // ADC High byte
        DEBUGF("L: %d\n", reg.data1);
        DEBUGF("H: %d\n", reg.data2);
        DEBUGF("ADC2 %d mV\n", (int)((double)( (reg.data2<<8)+reg.data1 )*6.4453125) );
        reg.status=0x00; // No error
       }
#endif
      } else if(reg.data0==3) { // Read Temp
#if defined(ADCTEMP)
       DEBUGF("- Temp\n");
       adc_init();
       ADMUX = (1<<REFS1) | (1<<REFS0) | (0<<ADLAR) | (1<<MUX3) | (0<<MUX2) | (0<<MUX1) | (0<<MUX0);
       ADCSRA |= (1<<ADSC);
       while(ADCSRA & (1<<ADSC));
       ADCSRA |= (1<<ADSC);
       while(ADCSRA & (1<<ADSC));
       DEBUGF("TEMP %f\n", (ADC - 324.31)/1.22);
       reg.data0 = ADCTEMP;
       reg.data1 = ADCL; // ADC Low byte
       reg.data2 = ADCH; // ADC High byte
       DEBUGF("L: %d\n", reg.data1);
       DEBUGF("H: %d\n", reg.data2);
       DEBUGF("TEMP %f\n", ((double)((reg.data2<<8)+reg.data1)  - 324.31)/1.22);
       reg.status=0x00; // No error
       adc_init(); // Set ADC back to normal ready for the next read
#endif
      }
      reg.cmd=0; // DONE
     } else if (reg.cmd==I2C_CMD_SET_ORDER) { // Set "order"
      if(reg.data0>0) {
       reg.order = reg.data0;
       DEBUGF("Set order %d\n", reg.order);
       reg.status=0x00; // No error
      } else {
       reg.status=0x01; // Error ("order" below 1)
      }
      reg.cmd=0; // DONE
     } else if (reg.cmd==I2C_CMD_GETPATH) { // Get USB path to Px (data0=x or 0=controller)
	tmpu8 = reg.data0;
#if !defined(NOPATHS)
	if(tmpu8>CTRL_MAXPI) { // Does data exist for this entry?
#endif
	 reg.cmd=0; // DONE
	 reg.data0 = reg.data1 = reg.data2 = reg.data3 = \
	 reg.data4 = reg.data5 = reg.data6 = reg.data7 = 255;
	 reg.status=0x01; // Error
#if !defined(NOPATHS)
	} else {
	 reg.data0 = paths[tmpu8][7];
	 reg.data1 = paths[tmpu8][6];
	 reg.data2 = paths[tmpu8][5];
	 reg.data3 = paths[tmpu8][4];
	 reg.data4 = paths[tmpu8][3];
	 reg.data5 = paths[tmpu8][2];
	 reg.data6 = paths[tmpu8][1];
	 reg.data7 = paths[tmpu8][0];
	 reg.cmd=0; // DONE
	 reg.status=0x00; // No error
	}
#endif
     } else { // Unknown command
	DEBUGF("Unknown command\n");
	reg.cmd=0; // DONE
	reg.status=0x03; // Error unknown command
     }
    }

#ifdef LOOP_CALL // Custom function defined in config.h/c if needed
    loop_call();
#endif
  }

  return 0;
}

/* ------------------------------------------------------------------------- */
