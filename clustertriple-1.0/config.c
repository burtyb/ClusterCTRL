#include "config.h"
/*
 * USB Paths to devices
 */

const unsigned char paths[CTRL_MAXPI+1][8] = {
	{ 1, 255, 255, 255, 255, 255, 255, 255 }, /* ClusterCTRL */
	{ 4, 255, 255, 255, 255, 255, 255, 255 }, /* P1 */
	{ 3, 255, 255, 255, 255, 255, 255, 255 }, /* P2 */
	{ 2, 255, 255, 255, 255, 255, 255, 255 }, /* P3 */
};

volatile uint8_t *pled[CTRL_MAXPI][2] = {
        // PORT, PIN
        { &PORTB, (uint8_t *)PB1 }, // P1LED
        { &PORTB, (uint8_t *)PB2 }, // P2LED
	{ &PORTB, (uint8_t *)PB5 }, // P3LED
};
volatile uint8_t *p[CTRL_MAXPI][2] = {
        // PORT, PIN
        { &PORTC, (uint8_t *)PC5 }, // P1
        { &PORTC, (uint8_t *)PC4 }, // P2
	{ &PORTC, (uint8_t *)PC0 }, // P3
};
volatile uint8_t *usbboot[CTRL_MAXPI][2] = {
	// PORT, PIN
	{ &PORTC, (uint8_t *)PC3 }, // P1
	{ &PORTC, (uint8_t *)PC2 }, // P2
	{ &PORTC, (uint8_t *)PC1 }, // P3
};
