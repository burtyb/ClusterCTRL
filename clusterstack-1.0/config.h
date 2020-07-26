#ifndef __config_h_included__
#define __config_h_included__

#include <avr/pgmspace.h>

// Hardware configuration

extern unsigned char fanstatus;

# define CTRL_TYPE       0x04 // 0x04=Cluster Stack

# define GPIOA // Use GPIO array for LED/Px
# undef GPIOD // Use GPIO defines for LED/Px
# undef GPIOE // Use I2C I/O expander

# define LEDPWRPORT PORTB
# define LEDPWRPIN  PB3
# undef LEDALERTPORT
# undef LEDALERTPIN
# undef HUBENPORT
# undef HUBENPIN
# undef HUBENHIGH
# define FANSTATUS // Keep fan status in variable
# define NOPATHS // Nodes are not connected to a USB hub with the controller
# define NOPLED // Nodes do not have individual power LED

#define AUTOONDELAY 1000 // ms to delay for Px auto power on

# undef FANENPORT
# undef FANENPIN

# define DDRBINIT ((0<<PB0)|(0<<PB1)|(1<<PB2)|(1<<PB3)|(1<<PB4)|(1<<PB5))
# define DDRCINIT ((1<<PC0)|(1<<PC1)|(1<<PC2)|(1<<PC3)|(1<<PC4)|(1<<PC5))
# define DDRDINIT ((0<<PD4)|(0<<PD5)|(0<<PD6)|(0<<PD7))

# define CTRL_MAXPI     5       // maximum number of Nodes we control

# define UARTDEBUG // Hardware supports debug UART
# undef USBBOOT // usbboot(rpiboot) supported?

# define ADCEN  	1 // Enable 2 ADC
# define ADC1   	6 // 5V/2
# define ADC2   	7 // VIN
# define ADC1TYPE	1 // Type 1 = 3v3 reference, sample divided by 2
# define ADC2TYPE	2 // Type 2 = 3v3 ref, sample 10k+1.07k divider (*0.0966576332429991)
# define ADCTEMP	2 // AVR supports getting internal temp

volatile uint8_t *p[CTRL_MAXPI][2];

// Custom loop call to check temp alerts from nodes and turn fans on as needed
#define LOOP_CALL
void loop_call( void );

#endif
