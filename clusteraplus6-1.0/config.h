#ifndef __config_h_included__
#define __config_h_included__

#include <avr/pgmspace.h>

// Hardware configuration

# define CTRL_TYPE       0x03 // 0x00=Cluster DA, 0x01=Cluster pHAT ...

# define GPIOA // Use GPIO array for LED/Px
# undef GPIOD // Use GPIO defines for LED/Px
# undef GPIOE // Use I2C I/O expander

# define LEDPWRPORT PORTB
# define LEDPWRPIN  PB0
# define LEDALERTPORT PORTB
# define LEDALERTPIN  PB1
# define HUBENPORT PORTB
# define HUBENPIN  PB4
# define LEDACTPORT PORTC
# define LEDACTPIN  PC2
# undef HUBENHIGH // Not defined as HUB is on when low

//# define I2C_PORT   PORTB
//# define I2C_PIN    PINB
//# define I2C_DDR    DDRB
//# define I2C_SDA    _BV(0)
//# define I2C_SCL    _BV(1)

# define FANENPORT PORTD
# define FANENPIN  PD5

# define DDRBINIT ((1<<PB0)|(1<<PB1)|(1<<PB2)|(1<<PB3)|(1<<PB4)|(1<<PB5))
# define DDRCINIT ((1<<PC0)|(1<<PC1)|(1<<PC2)|(1<<PC3)|(1<<PC4)|(1<<PC5))
# define DDRDINIT ((1<<PD4)|(1<<PD5)|(1<<PD6)|(1<<PD7))

# define CTRL_MAXPI     5       // maximum number of Pi Zeros we control

# define UARTDEBUG // Hardware supports debug UART
# undef USBBOOT // usbboot(rpiboot) supported?

# define ADCEN  2 // Enable 2 ADC
//# define ADC1 3 // Rev2
# define ADC1   6 // 5V/2 C+P1/2
# define ADC1TYPE	1
//# define ADC2 2 // Rev2
# define ADC2   7 // 5V/2 P3/4/5
# define ADC2TYPE	1
# define ADCTEMP        2 // AVR supports getting internal temp

// Data for paths to USB devices

extern const unsigned char paths[CTRL_MAXPI+1][8];

volatile uint8_t *pled[CTRL_MAXPI][2];
volatile uint8_t *p[CTRL_MAXPI][2];

#endif
