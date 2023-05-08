/*! \file */
/*!
 * servoDriver.c
 *
 * Description: Servo motor driver for MSP432P4111 Launchpad.
 *              Assumes SMCLK configured with 48MHz HFXT as source.
 *              Uses Timer_A2 and P5.6 (TA2.1)
 *
 *  Created on:
 *      Author:
 */

#include "servoDriver.h"
#include "msp.h"

/* Global Variables  */
uint16_t pulseWidthTicks = SERVO_MIN_ANGLE;


void initServoMotor(void) {
    // DONE configure servo pin (P5.6) for primary module function (TA2.1),
    //  output, initially LOW
    P5->SEL0 |= BIT6;
    P5->SEL1 &= ~BIT6;
    P5->OUT &= ~BIT6;
    P5->DIR |= BIT6;


    /* Configure Timer_A2 and CCR1 */
    // Set period of Timer_A2 in CCR0 register for Up Mode
    TIMER_A2->CCR[0] = SERVO_TMR_PERIOD;
    // Set initial positive pulse-width of PWM in CCR1 register
    TIMER_A2->CCR[1] = SERVO_MIN_ANGLE;

    // DONE configure TA2CCR1 for Compare mode, Reset/Set output mode, with interrupt disabled
    TIMER_A2->CCTL[1] = 0x08E0; //0b 00xx 1xx0 1110 x000 = 0x08E0
    // Configure Timer_A2 in Up Mode, with source SMCLK, prescale 8:1, and
    //  interrupt disabled  -  tick rate will be 1MHz (for SMCLK = 48MHz)
    // DONE configure Timer_A2 (requires setting control AND expansion register)
    TIMER_A2->CTL = 0x0294; // 0b xxxx xx10 1001 x100 = 0x0294
    TIMER_A2->EX0 |= 0x0004; //0b0100 = 0x0004 //So a total of 1:8:5 (1:40) pre-scale now
}

void incrementTenDegree(void) {
    // update pulse-width for <current angle> + <10 degrees>
    pulseWidthTicks += TEN_DEGREE_TICKS;
    if (pulseWidthTicks > SERVO_MAX_ANGLE) {
        pulseWidthTicks = SERVO_MIN_ANGLE;
    }
    // DONE update CCR1 register to set new positive pulse-width
    TIMER_A2->CCR[1] = pulseWidthTicks;
}

void setServoAngle(uint8_t angle) {
    // NOT NEEDED FOR EXERCISE - but would be useful function for driver
    TIMER_A2->CCR[1] = SERVO_MIN_ANGLE + (angle * ONE_DEGREE_TICKS);
}
