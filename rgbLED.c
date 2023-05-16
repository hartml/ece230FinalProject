/*! \file */
/*!
 * rgbLED.c
 *
 * Description: RGB driver for LED2 on MSP432P4111 Launchpad
 *
 *  Created on: 3/23/23
 *      Author: Jacob Teaney
 */

#include "msp.h"
#include "rgbLED.h"
#include <stdint.h>
#include <stdbool.h>


void RGBLED_init(void) {
    // DONE set LED2 pins as GPIO outputs
//    MOV     R0, #0
//    LDR     R1, P2SEL0_2
//    STRB    R0, [R1]
//    LDR     R1, P2SEL1_2
//    STRB    R0, [R1]
//    LDR     R1, P2DIR_2
//    ORR     R0, R0, #0x01
//    STRB    R0, [R1]
//
//    LDR R1, P2OUT_2
//    AND R0, R0, #0x00
//    STRB R0, [R1]
    RGB_PORT->SEL0 = (RGB_PORT->SEL0) & (~RGB_ALL_PINS);
    RGB_PORT->SEL1 = (RGB_PORT->SEL1) & (~RGB_ALL_PINS);
    RGB_PORT->DIR = (RGB_PORT->DIR) | RGB_ALL_PINS;

    // DONE set LED2 outputs to LOW
    RGB_PORT->OUT = (RGB_PORT->OUT) & (~RGB_ALL_PINS);
}

void RGBLED_toggleRed(void) {
    // XOR bit 0 of Port 2 for red LED
    RGB_PORT->OUT = (RGB_PORT->OUT) ^ RGB_RED_PIN;
}

void RGBLED_toggleGreen(void) {
    // DONE toggle bit 1 of Port 2 OUT register
    RGB_PORT->OUT = (RGB_PORT->OUT) ^ RGB_GREEN_PIN;
}

void RGBLED_toggleBlue(void) {
    // DONE toggle bit 2 of Port 2 OUT register
    RGB_PORT->OUT = (RGB_PORT->OUT) ^ RGB_BLUE_PIN;
}

void RGBLED_toggleYellow(void) {
    RGB_PORT->OUT = (RGB_PORT->OUT) ^ RGB_YELLOW_PIN;
}

void RGBLED_setColor(Colors color) {
    // DONE set the R, G, and B output values based on color value
    switch(color){
    case(BLACK):
        RGB_PORT->OUT = (RGB_PORT->OUT) & (~RGB_RED_PIN);
        RGB_PORT->OUT = (RGB_PORT->OUT) & (~RGB_GREEN_PIN);
        RGB_PORT->OUT = (RGB_PORT->OUT) & (~RGB_BLUE_PIN);
        break;
    case(RED):
        RGB_PORT->OUT = (RGB_PORT->OUT) | (RGB_RED_PIN);
        RGB_PORT->OUT = (RGB_PORT->OUT) & (~RGB_GREEN_PIN);
        RGB_PORT->OUT = (RGB_PORT->OUT) & (~RGB_BLUE_PIN);
        break;
    case(GREEN):
        RGB_PORT->OUT = (RGB_PORT->OUT) & (~RGB_RED_PIN);
        RGB_PORT->OUT = (RGB_PORT->OUT) | (RGB_GREEN_PIN);
        RGB_PORT->OUT = (RGB_PORT->OUT) & (~RGB_BLUE_PIN);
        break;
    case(YELLOW):
        RGB_PORT->OUT = (RGB_PORT->OUT) | (RGB_RED_PIN);
        RGB_PORT->OUT = (RGB_PORT->OUT) | (RGB_GREEN_PIN);
        RGB_PORT->OUT = (RGB_PORT->OUT) & (~RGB_BLUE_PIN);
        break;
    case(BLUE):
        RGB_PORT->OUT = (RGB_PORT->OUT) & (~RGB_RED_PIN);
        RGB_PORT->OUT = (RGB_PORT->OUT) & (~RGB_GREEN_PIN);
        RGB_PORT->OUT = (RGB_PORT->OUT) | (RGB_BLUE_PIN);
        break;
    case(MAGENTA):
        RGB_PORT->OUT = (RGB_PORT->OUT) | (RGB_RED_PIN);
        RGB_PORT->OUT = (RGB_PORT->OUT) & (~RGB_GREEN_PIN);
        RGB_PORT->OUT = (RGB_PORT->OUT) | (RGB_BLUE_PIN);
        break;
    case(CYAN):
        RGB_PORT->OUT = (RGB_PORT->OUT) & (~RGB_RED_PIN);
        RGB_PORT->OUT = (RGB_PORT->OUT) | (RGB_GREEN_PIN);
        RGB_PORT->OUT = (RGB_PORT->OUT) | (RGB_BLUE_PIN);
        break;
    case(WHITE):
        RGB_PORT->OUT = (RGB_PORT->OUT) | (RGB_RED_PIN);
        RGB_PORT->OUT = (RGB_PORT->OUT) | (RGB_GREEN_PIN);
        RGB_PORT->OUT = (RGB_PORT->OUT) | (RGB_BLUE_PIN);
        break;
    default:
        RGB_PORT->OUT = (RGB_PORT->OUT) & (~RGB_RED_PIN);
        RGB_PORT->OUT = (RGB_PORT->OUT) & (~RGB_GREEN_PIN);
        RGB_PORT->OUT = (RGB_PORT->OUT) & (~RGB_BLUE_PIN);
        break;
    };
}
