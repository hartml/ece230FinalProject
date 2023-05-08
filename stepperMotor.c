/*! \file */
/*!
 * stepperMotor.c
 *
 * Description: Stepper motor ULN2003 driver for MSP432P4111 Launchpad.
 *              Assumes SMCLK configured with 48MHz HFXT as source.
 *              Uses Timer_A3 and P2.7, P2.6, P2.5, P2.4
 *
 *  Created on:
 *      Author:
 */

#include "stepperMotor.h"
#include "msp.h"

/* Global Variables  */
// DONE fill in array with 4-bit binary sequence for wave drive (1-phase full step)
const uint8_t stepperSequence[STEP_SEQ_CNT] = {0b0001, 0b0011, 0b0010, 0b0110, 0b0100, 0b1100, 0b1000, 0b1001};
uint16_t stepPeriod = INIT_PERIOD;
uint8_t currentStep = 0;
bool GoReverse = true;
static int64_t R0Value = 0;

void initStepperMotor(void) {
    // set stepper port pins as GPIO outputs
    STEPPER_PORT->SEL0 = (STEPPER_PORT->SEL0) & ~STEPPER_MASK;
    STEPPER_PORT->SEL1 = (STEPPER_PORT->SEL1) & ~STEPPER_MASK;
    STEPPER_PORT->DIR = (STEPPER_PORT->DIR) | STEPPER_MASK;

    // initialize stepper outputs to LOW
    STEPPER_PORT->OUT = (STEPPER_PORT->OUT) & ~STEPPER_MASK;

    /* Configure Timer_A3 and CCR0 */
    // Set period of Timer_A3 in CCR0 register for Up Mode
    TIMER_A3->CCR[0] = stepPeriod;
    // DONE configure CCR0 for Compare mode with interrupt enabled (no output mode - 0)
    TIMER_A3->CCTL[0] = 0x0810; //00xx 10x0 xxx1 xx00

    // Configure Timer_A3 in Stop Mode, with source SMCLK, prescale 18:1, and
    //  interrupt disabled  -  tick rate will be 2MHz (for SMCLK = 48MHz)
    // DONE configure Timer_A3 (requires setting control AND expansion register)
    TIMER_A3->CTL = 0x0286; //xxxx xx10 1000 x110 //1:4
    TIMER_A3->EX0 |= 0x0005; //Add a 6:1 Pre-scale

    /* Configure global interrupts and NVIC */
    // Enable TA3CCR0 compare interrupt by setting IRQ bit in NVIC ISER0 register
    // DONE enable interrupt by setting IRQ bit in NVIC ISER0 register
    NVIC_EnableIRQ(TA3_0_IRQn);

    __enable_irq();                             // Enable global interrupt
}

void enableStepperMotor(void) {
    // DONE configure Timer_A3 in Up Mode (leaving remaining configuration unchanged)
    TIMER_A3->CTL |= 0x0010; //xxxx xxxx xx01 xxxx
    TIMER_A3->CTL &= ~(0x0020);
}

void disableStepperMotor(void) {
    //  Configure Timer_A3 in Stop Mode (leaving remaining configuration unchanged)
    TIMER_A3->CTL &= ~(0x0030); //xxxx xxxx xx00 xxxx
}

void stepClockwise(void) {
    currentStep = (currentStep + 1) % STEP_SEQ_CNT;  // increment to next step position
    STEPPER_PORT->OUT = (STEPPER_PORT->OUT & ~(0x00F0)) | (stepperSequence[currentStep] << 4);
}

void stepCounterClockwise(void) {
    currentStep = ((uint8_t)(currentStep - 1)) % STEP_SEQ_CNT;  // decrement to previous step position (counter-clockwise)
    STEPPER_PORT->OUT = (STEPPER_PORT->OUT & ~(0x00F0)) | (stepperSequence[currentStep] << 4);
}

void setRPM(bool goReverse, uint16_t rpm100x) {
    R0Value = (5859375) / rpm100x; //((rpm/60)*2048)^(-1)*(2000000) The eqn for the CCR[0] to get x rpm
    TIMER_A3->CCR[0] = R0Value; //Assuming rpm is 100x what it should be (to keep it as an integer).
    GoReverse = goReverse;
}

// Timer A3 CCR0 interrupt service routine
void TA3_0_IRQHandler(void) {
    /* Not necessary to check which flag is set because only one IRQ
     *  mapped to this interrupt vector     */
    if(GoReverse) {
        stepCounterClockwise();
    } else {
        stepClockwise();
    }
    // DONE clear timer compare flag in TA3CCTL0
    TIMER_A3->CCTL[0] &= ~0x0001;
}
