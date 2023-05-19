/*
 * IRReceiver.c
 *
 *  Created on: May 15, 2023
 *      Author: teaneyje
 */

#include <stdbool.h>
#include <math.h>
#include "msp.h"

#define PinInPort   P2
#define PinInBit    BIT4
#define OneMaxTime 2 //ms
#define OneMinTime 1 //ms
#define OneMaxTicks = 12000
#define OneMinTicks = 6000

bool ReadLastPressed = true;
char LastPressed = '\0';
uint16_t StartingTick = 0;
uint16_t EndingTick = 0;
bool ReadingArray[32];

void InitializeIRReceiver() {
    TIMER_A0->CTL = 0x02E4; //0b xxxx xx10 1110 x10x //Setting Timer A0 to have 1:8 pre-scale with SMCLK as source on continuous mode, no interrupts

    PinInPort->DIR &= ~(PinInBit); //Configuring (PinInPort).(PinInBit) as secondary function input with pull-down resistor
    PinInPort->SEL1 &= ~(PinInBit);
    PinInPort->SEL0 |= BIT4;
    PinInPort->REN &= ~BIT4;

    TIMER_A0->CCTL[1] = 0xC110; //0b 1100 0xx1 xxx1 xxxx //Setting CCR1 to be capture on rising and falling with interrupts enabled

    NVIC_EnableIRQ(TA0_N_IRQn);
    __enable_irq();                             // Enable global interrupt
}

char getLastPressed() {
    if(!ReadLastPressed) {
        ReadLastPressed = true;
        return LastPressed;
    }
    return '\0';
}

void handleDecodingProcess() {
    int tmp = 0;
    int i = 0;
    for(i = 16; i < 23; i++) { //Looking at [16, 23) since these are the data bits
        if(ReadingArray[i]) {
            int j = i - 16;
            tmp += powf(2, j);
        }
    }
    LastPressed = tmp;
}

void handleNewBit() {
    uint16_t ticks = EndingTick - StartingTick;
    static uint16_t index = 31;
    int i = 0;
    if(ticks > 12000) { //The pulse was the starting sequence. So we need to start overwriting our current reading resister
        for(i = 0; i < 32; i++ ) {
            ReadingArray[i] = false;
        }
        index = 0;
    } else if(ticks < 6000) { //The pulse was a 0
        //Add a 0 to the reading array
        ReadingArray[i] = false;
    } else { //The pulse was a 1
        //Add 1 to the reading register
        ReadingArray[i] = true;
    }

    if(index == 0) {
        handleDecodingProcess();
        ReadLastPressed = false;
    }
    index--;
}

void TA0_N_IRQHandler() {
    //If the input pin is a 1 (so a rising triggered this interrupt), we need to start counting. So set Starting tick to be TA0R
    //Else, this was a falling edge that caused this interrupt, so we need to stop counting (set the ending tick to be TA0R), and calculate if it was a 0, 1, or one of the starting bits
    if(PinInPort->IN & BIT4 == BIT4) { //The input is a 1
        TIMER_A0->R = 0; //Resetting it, so we don't get an overflow issue or anything while reading the signal
        StartingTick = 0;
    } else {
        EndingTick = TIMER_A0->R;
        handleNewBit();
    }
    TIMER_A0->CCTL[1] &= ~(BIT1 | BIT);
}
