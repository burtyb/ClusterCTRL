#ifndef __config_h_included__
#define __config_h_included__

#include <avr/pgmspace.h>

// Hardware configuration

# define CTRL_TYPE       0x02 // 0x00=Cluster DA, 0x01=Cluster Triple, ...

# define ENP1PORT PORTD
# define ENP1PIN  PD0
# define ENP2PORT PORTD
# define ENP2PIN  PD5
# define LEDP1PORT PORTB
# define LEDP1PIN  PB1
# define LEDP2PORT PORTB
# define LEDP2PIN  PB0
# define LEDPWRPORT PORTB
# define LEDPWRPIN  PB3
# undef LEDACTPORT
# undef LEDACTPIN
# define LEDALERTPORT PORTB
# define LEDALERTPIN  PB2
# define HUBENPORT PORTD
# define HUBENPIN  PD1

# define IO_PLED
# define IO_P

# define HUBENHIGH // Hub is on when high
# undef UARTDEBUG // Hardware supports debug UART
# define CTRL_MAXPI      2      // maximum number of Pi Zeros we control
# undef USBBOOT // usbboot(rpiboot) supported?

# define DDRBINIT ((1<<PB0)|(1<<PB1)|(1<<PB2)|(1<<PB3))
# define DDRDINIT ((1<<PD1)|(1<<PD2)|(1<<PD5))

// Data for paths to USB devices

extern const unsigned char paths[CTRL_MAXPI+1][8];
volatile uint8_t *pled[CTRL_MAXPI][2];
volatile uint8_t *p[CTRL_MAXPI][2];

#endif
