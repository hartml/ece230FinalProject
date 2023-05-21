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
#define OneMax      1500 //2 ms
#define OneMin      7500 //1 ms
#define ZeroMin     356 //475 us
#define InitialMin  6750 //9 ms
#define InitialMax  6900 //9.2 ms
#define SecondMin   3150 //4.2 ms
#define SecondMax   3450 //4.6 ms
#define LargeOneMin 1125
#define LargeOneMax 2025
#define LargeZeroMin 750
#define LargeZeroMax 900
#define LargeStartingMin 6500
#define LargeStartingMax 7500

bool ReadLastPressed = true;
char LastPressed = '\0';
uint16_t RisingTick = 0;
uint16_t FallingTick = 0;
bool TimerHasOverflowed = true;
bool ReadingArray[32];
static int16_t index = 32;
uint16_t CurrentTick = 0;
uint16_t PreviousTick = 0;

void InitializeIRReceiver() {
    TIMER_A0->CTL = 0x02E4; //0b xxxx xx10 1110 x10x //Setting Timer A0 to have 1:8 pre-scale with SMCLK as source on continuous mode, no interrupts
//    TIMER_A0->EX0 |= 0x0007; //Adding a 1:8 prescale so that the total is 1:8:8 so a 0.75 MHz clock source

    PinInPort->DIR &= ~(PinInBit); //Configuring (PinInPort).(PinInBit) as secondary function input with pull-down resistor
    PinInPort->SEL1 &= ~(PinInBit);
    PinInPort->SEL0 |= BIT4;
    PinInPort->REN &= ~BIT4;

    TIMER_A0->CCTL[1] = 0x4110; //0b 1100 0xx1 xxx1 xxxx //Setting CCR1 to be capture on rising and falling with interrupts enabled

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

bool isOne() {
    int16_t ticks = CurrentTick - PreviousTick;
    if(ticks < LargeOneMax && ticks > LargeOneMin) {
        return true;
    }
    return false;
}

bool isZero() {
    int16_t ticks = CurrentTick - PreviousTick;
    if(ticks < LargeZeroMax && ticks > LargeZeroMin) {
        return true;
    }
    return false;
}

bool isStartingCondition() {
    int16_t ticks = CurrentTick - PreviousTick;
    if(ticks < LargeStartingMax && ticks > LargeStartingMin) {
        return true;
    }
    return false;
}

void OnFalling() {
    PreviousTick = CurrentTick;
    CurrentTick = TIMER_A0->R;
    if(isStartingCondition()) {
        index = 31;
        TimerHasOverflowed = false;
        TIMER_A0->R = 0;
        return;
    } else if(TimerHasOverflowed) {
        return; //Noise
    } else if(isOne()) {
        ReadingArray[index] = true;
    } else if(isZero()) {
        ReadingArray[index] = false;
    } else {
        return; //Should never be getting called
    }

    if(index == 0) {
        handleDecodingProcess();
        TimerHasOverflowed = true; //Not actually happening, but should help counter possible noise
        return;
    }

    index--;
}

void TA0_N_IRQHandler() {
    OnFalling();
    TIMER_A0->CCTL[1] &= ~(BIT1 | BIT2);
}
