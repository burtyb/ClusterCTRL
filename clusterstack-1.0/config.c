#include "config.h"

unsigned char fanstatus;

volatile uint8_t *p[CTRL_MAXPI][2] = {
        // PORT, PIN
        { &PORTC, (uint8_t *)PC2 }, // P1
        { &PORTC, (uint8_t *)PC3 }, // P2
	{ &PORTC, (uint8_t *)PC4 }, // P3
	{ &PORTC, (uint8_t *)PC5 }, // P4
	{ &PORTB, (uint8_t *)PB2 }, // P5
};

void loop_call( void ) {
	// Check if P1/P2 are requesting cooling
	if(((PINB & (1<<PINB1))>>PINB1)|((PINB & (1<<PINB0))>>PINB0)|fanstatus) {
		PORTC |= (1<<PC0); // Turn fan1 on
	} else {
		PORTC &= ~(1<<PC0); // Turn fan1 off
	}

	// Check if P3/P4/P5 want the fan turning on
	if(((PIND & (1<<PIND7))>>PIND7)|((PIND & (1<<PIND6))>>PIND6)|((PIND & (1<<PIND5))>>PIND5)|((PIND & (1<<PIND4))>>PIND4)|fanstatus) {
		PORTC |= (1<<PC1); // Turn fan2 on
	} else {
		PORTC &= ~(1<<PC1); // Turn fan2 off
	}
}

uint8_t get_fanstatus( void ) {
	uint8_t data = 0;
	data |= ((PINB & (1<<PINB0))>>PINB0)<<0; // Status P1 requested cooling
	data |= ((PINB & (1<<PINB1))>>PINB1)<<1; // Status P2 requested cooling
	data |= ((PIND & (1<<PIND6))>>PIND6)<<2; // Status P3 requested cooling
	data |= ((PIND & (1<<PIND7))>>PIND7)<<3; // Status P4 requested cooling
	data |= ((PIND & (1<<PIND4))>>PIND4)<<4; // Status P5 requested cooling
	data |= ((PIND & (1<<PIND5))>>PIND5)<<5; // Status <spare> requested cooling
	
	return data;
}
