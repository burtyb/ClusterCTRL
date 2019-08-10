#include "config.h"
/*
 * USB Paths to devices
 */

const unsigned char paths[CTRL_MAXPI+1][8] = {
	{ 4, 1, 255, 255, 255, 255, 255, 255 }, /* ClusterCTRL */
	{ 1, 255, 255, 255, 255, 255, 255, 255 }, /* P1 */
	{ 2, 255, 255, 255, 255, 255, 255, 255 }, /* P2 */
	{ 3, 255, 255, 255, 255, 255, 255, 255 }, /* P3 */
	{ 4, 2, 255, 255, 255, 255, 255, 255 }, /* P4 */
	{ 4, 4, 255, 255, 255, 255, 255, 255 }, /* P5 */
};

volatile uint8_t *pled[CTRL_MAXPI][2] = {
        // PORT, PIN
        { &PORTB, (uint8_t *)PB2 }, // P1LED
        { &PORTB, (uint8_t *)PB5 }, // P2LED
	{ &PORTC, (uint8_t *)PC3 }, // P3LED
	{ &PORTC, (uint8_t *)PC0 }, // P4LED
	{ &PORTC, (uint8_t *)PC1 }, // P5LED
};
volatile uint8_t *p[CTRL_MAXPI][2] = {
        // PORT, PIN
        { &PORTD, (uint8_t *)PD7 }, // P1
        { &PORTD, (uint8_t *)PD6 }, // P2
	{ &PORTD, (uint8_t *)PD4 }, // P3
	{ &PORTC, (uint8_t *)PC5 }, // P4
	{ &PORTC, (uint8_t *)PC4 }, // P5
};
