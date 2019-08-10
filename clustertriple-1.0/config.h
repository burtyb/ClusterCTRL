#ifndef __config_h_included__
#define __config_h_included__

#include <avr/pgmspace.h>

// Hardware configuration

# define CTRL_TYPE       0x01 // 0x00=Cluster DA, 0x01=Cluster pHAT ...

# define LEDPWRPORT PORTD
# define LEDPWRPIN  PD4
# define LEDALERTPORT PORTD
# define LEDALERTPIN  PD7
# define HUBENPORT PORTB
# define HUBENPIN  PB4
# undef HUBENHIGH // Not defined as HUB is on when low

# define FANENPORT PORTD
# define FANENPIN  PD6

# define DDRBINIT ((1<<PB0)|(1<<PB1)|(1<<PB2)|(1<<PB3)|(1<<PB4)|(1<<PB5))
# define DDRCINIT ((1<<PC0)|(1<<PC1)|(1<<PC2)|(1<<PC3)|(1<<PC4)|(1<<PC5))
# define DDRDINIT ((1<<PD4)|(1<<PD5)|(1<<PD6)|(1<<PD7))

# define CTRL_MAXPI     3       // maximum number of Pi Zeros we control

# define UARTDEBUG // Hardware supports debug UART
# define USBBOOT // usbboot(rpiboot) supported?

# define ADCEN  	1 // Enable 2 ADC
# define ADC1   	6 // 5V/2
# define ADC1TYPE	1 // Type 1 = 3v3 reference, sample divided by 2
# define ADCTEMP	2 // AVR supports getting internal temp

# define GPIOCTRL // Use GPIO for LED/Px

// Data for paths to USB devices
extern const unsigned char paths[CTRL_MAXPI+1][8];
volatile uint8_t *pled[CTRL_MAXPI][2];
volatile uint8_t *p[CTRL_MAXPI][2];
volatile uint8_t *usbboot[CTRL_MAXPI][2];

#endif
