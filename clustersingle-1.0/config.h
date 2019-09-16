#ifndef __config_h_included__
#define __config_h_included__

#include <avr/pgmspace.h>

// Hardware configuration

# define CTRL_TYPE       0x03 // 0x00=Cluster DA, 0x01=Cluster Triple, ...

# undef GPIOA // Use GPIO array for LED/Px
# define GPIOD // Use GPIO defines for LED/Px
# undef GPIOE // Use I2C I/O expander


# define ENP1PORT PORTD
# define ENP1PIN  PD0

# define LEDP1PORT PORTB
# define LEDP1PIN  PB4

# define LEDPWRPORT PORTB
# define LEDPWRPIN  PB2
# undef LEDACTPORT
# undef LEDACTPIN
# define LEDALERTPORT PORTB
# define LEDALERTPIN  PB3
# define HUBENPORT PORTB
# define HUBENPIN  PB0
# undef HUBENHIGH // Not defined as HUB is on when low

# define USBBOOT        // usbboot(rpiboot) supported?
# define USBBOOTP1PORT	PORTD
# define USBBOOTP1PIN	PD1

# undef UARTDEBUG 		// Hardware supports debug UART
# define CTRL_MAXPI	1	// maximum number of Pi Zeros we control

# define DDRBINIT ((1<<PB0)|(1<<PB2)|(1<<PB3)|(1<<PB4))
# define DDRDINIT ((1<<PD0)|(1<<PD1))

// Data for paths to USB devices

extern const unsigned char paths[CTRL_MAXPI+1][8];

#endif
