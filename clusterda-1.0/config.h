#ifndef __config_h_included__
#define __config_h_included__

#include <avr/pgmspace.h>

// Hardware configuration

# define DDRBINIT ((1<<PB0)|(1<<PB1)|(1<<PB2)|(1<<PB3)|(1<<PB4)|(1<<PB5))
# define DDRCINIT ((1<<PC0)|(1<<PC1)|(1<<PC2)|(1<<PC3)|(1<<PC4)|(1<<PC5))
# define DDRDINIT ((1<<PD4)|(1<<PD5)|(1<<PD6)|(1<<PD7))


# define CTRL_TYPE	0x00

# undef GPIOA // Use GPIO array for LED/Px
# undef GPIOD // Use GPIO defines for LED/Px
# define GPIOE // Use I2C I/O expander

# define UARTDEBUG // Hardware supports debug UART

# define I2C_HW		1
# define I2C_PORT	PORTC
# define I2C_PIN	PINC
# define I2C_DDR	DDRC
# define I2C_SDA	_BV(4)
# define I2C_SCL	_BV(5)

// Which ports of the MUX can a HAT be on (bit field)
#define HATONMUX        0xFF // All ports

#define LEDPWRPORT	PORTB
#define LEDPWRPIN	PB1
#undef LEDACTPORT
#undef LEDACTPIN
#define LEDALERTPORT	PORTB
#define LEDALERTPIN	PB2

// I2C Multiplexer (!RESET)
#define MUXRESETPORT	PORTD
#define MUXRESETPIN	PD6
#define MUXRESETHIGH

// 3v3 regulator for EEPROM/Expander (EN)
#define REG1PORT	PORTD
#define REG1PIN		PD5
#define REG1HIGH	

/* Address of devices */
# define ADDR_MUX 0x70 // I2C Address of multiplexer
# define ADDR_PRI 0x20 // I2C Address of expander on v2.x HAT
# define ADDR_SEC 0x21 // I2C Address of expander on adapter
# define ADDR_EEPROM 0x50 // I2C Address of HAT EEPROM

// TODO ADC1/ADC2
# define ADCEN
# define ADCTEMP       2 // AVR supports getting internal temp

// Registers for XRA1200P I2C Expander
#define XRA1200_IN   0
#define XRA1200_OUT  1
#define XRA1200_DIR  3
#define XRA1200_PUR  4

// Expander I/O bits to set/reset
#define X_P1    0
#define X_P2    1
#define X_P3    2
#define X_P4    3
#define X_LED   4
#define X_HUB   5
#define X_ALERT 6
#define X_WP    7

# define CTRL_MAXPI	32       // maximum number of Pi Zeros we control
# undef USBBOOT // usbboot(rpiboot) supported?

// Data for paths to USB devices
extern const unsigned char paths[CTRL_MAXPI+1][8];

#endif
