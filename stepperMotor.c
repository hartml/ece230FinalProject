/*! \file */
/*!
 * stepperMotor.c
 *
 * Description: Stepper motor ULN2003 driver for MSP432P4111 Launchpad.
 *              Assumes SMCLK configured with 48MHz HFXT as source.
 *              Uses Timer_A3 and P2.7, P2.6, P2.5, P2.4
 *
 *      Author: Matthew Hart
 */

#include "stepperMotor.h"
#include "msp.h"
#include <stdbool.h>

/* Global Variables  */
// fill in array with 4-bit binary sequence for half drive)
const uint8_t stepperSequence[STEP_SEQ_CNT] = {0b0001, 0b0011, 0b0010, 0b0110, 0b0100, 0b1100, 0b1000, 0b1001};
uint64_t initPeriod = MIN_PERIOD;
uint64_t maxPeriod = MAX_PERIOD;
uint8_t currentStep = 0;
bool reverseMode = false;



void initStepperMotor(void) {
    // set stepper port pins as GPIO outputs
    STEPPER_PORT->SEL0 = (STEPPER_PORT->SEL0 & ~STEPPER_MASK);
    STEPPER_PORT->SEL1 = (STEPPER_PORT->SEL1 & ~STEPPER_MASK);
    STEPPER_PORT->DIR = (STEPPER_PORT->DIR | STEPPER_MASK);

    // initialize stepper outputs to LOW
    STEPPER_PORT->OUT = (STEPPER_PORT->OUT & ~STEPPER_MASK);

    /* Configure Timer_A3 and CCR0 */
    // Set period of Timer_A3 in CCR0 register for Up Mode
    TIMER_A3->CCR[0] = maxPeriod;       // update Period based on Potentiometer value
    // configure CCR0 for Compare mode with interrupt enabled (no output mode - 0)
    TIMER_A3->CCTL[0] = 0b0000000000010000;
    // Configure Timer_A3 in Stop Mode, with source SMCLK, prescale 64:1, and
    //  interrupt disabled  -  tick rate will be 375kHz (for SMCLK = 48MHz)
    // configure Timer_A3 (requires setting control AND expansion register)
    TIMER_A3->CTL = 0b0000001011000000;
    TIMER_A3->EX0 = 0b0000000000000111;

    /* Configure global interrupts and NVIC */
    // Enable TA3CCR0 compare interrupt by setting IRQ bit in NVIC ISER0 register
    // enable interrupt by setting IRQ bit in NVIC ISER0 register

    NVIC->ISER[0] = (NVIC->ISER[0] | 0x4000);

    __enable_irq();                             // Enable global interrupt
}

void enableStepperMotor(void) { // enable stepper motor
    // configure Timer_A3 in Up Mode (leaving remaining configuration unchanged)
    TIMER_A3->CTL = (TIMER_A3->CTL | 0b0000000000010000);
    TIMER_A3->CTL = (TIMER_A3->CTL & 0b1111111111011111);
    stepClockwise();
}

void disableStepperMotor(void) {  // disable stepper motor
    //  Configure Timer_A3 in Stop Mode (leaving remaining configuration unchanged)
    while(1==1){}; // stay in disable Motor NOT WORKING
}

void stepClockwise(void) { // stepper motor clockwise rotation
    currentStep = ((currentStep + 1) % STEP_SEQ_CNT);  // increment to next step position
    // update output port for current step pattern
    STEPPER_PORT->OUT = ((STEPPER_PORT->OUT & 0x0F) + (stepperSequence[currentStep] << 4));
}

void stepCounterClockwise(void) { // stepper motor counterclockwise rotation
    currentStep = ((uint8_t)(currentStep - 1)) % STEP_SEQ_CNT;  // decrement to previous step position (counter-clockwise)
    //  update output port for current step pattern
    STEPPER_PORT->OUT = ((STEPPER_PORT->OUT & 0x0F) + (stepperSequence[currentStep] << 4));
}

// Timer A3 CCR0 interrupt service routine
void TA3_0_IRQHandler(void)
{
    if((P1->IN & 0b00010000) != 16){ // if S2 pressed
        SysTick->VAL = 10000;
        while((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)){} // debounce
        while((P1->IN & 0b00010000) != 16){} // wait until S2 is released
        SysTick->VAL = 10000;
        while((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)){} // debounce
            if(reverseMode == true){
                reverseMode = false;  // toggle reverseMode when S2 is pressed
            }
            else{
                reverseMode = true;
            }
        }
    if(reverseMode == true){
    stepCounterClockwise();
    }
    else{
    stepClockwise();
    }
    // TODO clear timer compare flag in TA3CCTL0
    TIMER_A3->CCTL[0] = (TIMER_A3->CCTL[0] & ~TIMER_A_CCTLN_CCIFG);
}
