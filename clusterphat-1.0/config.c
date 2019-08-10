#include "config.h"
/*
 * USB Paths to devices
 */

const unsigned char paths[CTRL_MAXPI+1][8] = {
        { 2, 255, 255, 255, 255, 255, 255, 255 }, /* ClusterCTRL */
        { 4, 255, 255, 255, 255, 255, 255, 255 }, /* P1 */
        { 3, 255, 255, 255, 255, 255, 255, 255 } /* P2 */
};

volatile uint8_t *pled[CTRL_MAXPI][2] = {
        // PORT, PIN
	{ &PORTB, (uint8_t *)PB1 }, // P1LED
	{ &PORTB, (uint8_t *)PB0 }, // P2LED
};
volatile uint8_t *p[CTRL_MAXPI][2] = {
        // PORT, PIN
        { &PORTD, (uint8_t *)PB0 }, // P1
        { &PORTD, (uint8_t *)PB5 }, // P2
};

