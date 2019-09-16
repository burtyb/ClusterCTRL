#include "config.h"
/*
 * USB Paths to devices
 */

const unsigned char paths[CTRL_MAXPI+1][8] = {
        { 2, 255, 255, 255, 255, 255, 255, 255 }, /* ClusterCTRL */
        { 4, 255, 255, 255, 255, 255, 255, 255 }, /* P1 */
};

volatile uint8_t *pled[CTRL_MAXPI+1][2] = {
        // PORT, PIN
	{ 0,0 },
	{ &PORTB, (uint8_t *)PB4 }, // P1LED
};
volatile uint8_t *p[CTRL_MAXPI+1][2] = {
        // PORT, PIN
	{ 0,0 },
        { &PORTD, (uint8_t *)PD0 }, // P1
};

volatile uint8_t *usbboot[CTRL_MAXPI+1][2] = {
        // PORT, PIN
	{ 0,0 },
        { &PORTD, (uint8_t *)PD1 }, // P1
};
