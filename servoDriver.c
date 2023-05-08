/*! \file */
/*!
 * servoDriver.c
 *
 * Description: Servo motor driver for MSP432P4111 Launchpad.
 *              Assumes SMCLK configured with 48MHz HFXT as source.
 *              Uses Timer_A2 and P5.6 (TA2.1)
 *
 *      Author: Matthew Hart
 */

#include "servoDriver.h"
#include "msp.h"

/* Global Variables  */
uint16_t pulseWidthTicks = SERVO_MIN_ANGLE;



void initServoMotor(void) {
    // configure servo pin (P5.6) for primary module function (TA2.1),
    //  output, initially LOW
    P5->SEL0 |= 0b01000000;
    P5->SEL1 &= 0b10111111;
    P5->DIR |=  0b01000000;
    P5->OUT &=  0b10111111;

    /* Configure Timer_A2 and CCR1 */
    // Set period of Timer_A2 in CCR0 register for Up Mode
    TIMER_A2->CCR[0] = SERVO_TMR_PERIOD;
    // Set initial positive pulse-width of PWM in CCR1 register
    TIMER_A2->CCR[1] = SERVO_MIN_ANGLE;

    // configure TA2CCR1 for Compare mode, Reset/Set output mode, with interrupt disabled
    TIMER_A2->CCTL[1] = 0b0000000011100000;

    // Configure Timer_A2 in Up Mode, with source SMCLK, prescale 64:1, and
    //  interrupt disabled  -  tick rate will be 375kHz (for SMCLK = 48MHz)
    // configure Timer_A2 (requires setting control AND expansion register)
    TIMER_A2->CTL = 0b0000001011010000; // 1:8 prescale
    TIMER_A2->EX0 = 0b0000000000000111; // 1:8 prescale expansion
}



