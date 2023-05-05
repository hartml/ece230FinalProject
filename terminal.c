/*
 * terminal.c
 *
 *  Created on: May 1, 2023
 *      Author: teaneyje
 */
#include "terminal.h"
#include "msp.h"

void initializeTerminal() {
    /* Configure MCLK/SMCLK source to DCO, with DCO = 12MHz */
    CS->KEY = CS_KEY_VAL;                   // Unlock CS module for register access
    CS->CTL0 = 0;                           // Reset tuning parameters
    CS->CTL0 = CS_CTL0_DCORSEL_3;           // Set DCO to 12MHz (nominal, center of 8-16MHz range)
    CS->CTL1 = CS_CTL1_SELA_2 |             // Select ACLK = REFO
            CS_CTL1_SELS_3 |                // SMCLK = DCO
            CS_CTL1_SELM_3;                 // MCLK = DCO
    CS->KEY = 0;                            // Lock CS module from unintended accesses

    /* Configure UART pins */
    P3->SEL0 |= BIT2 | BIT3;                // set 2-UART pins as secondary function
    P3->SEL1 &= ~(BIT2 | BIT3);

    /* Configure UART
     *  Asynchronous UART mode, 8N1 (8-bit data, no parity, 1 stop bit), //TODO: Make it 8E1
     *  LSB first, SMCLK clock source
     */
    EUSCI_A2->CTLW0 |= EUSCI_A_CTLW0_SWRST; // Put eUSCI in reset
    EUSCI_A2->CTLW0 = EUSCI_A_CTLW0_SWRST | // Remain eUSCI in reset
            EUSCI_A_CTLW0_SSEL__SMCLK;      // Configure eUSCI clock source for SMCLK

    /* Baud Rate calculation
     * Refer to Section 24.3.10 of Technical Reference manual
     * BRCLK = 12000000, Baud rate = 9600
     * N = fBRCLK / Baud rate = 12000000/9600 = 1250
     * from Technical Reference manual Table 24-5:
     *
     * DONE lookup values for UCOS16, UCBRx, UCBRFx, and UCBRSx in Table 24-5
     */
    // DONE set clock prescaler in EUSCI_A2 baud rate control register
    EUSCI_A2->BRW = 78;
    // DONE configure baud clock modulation in EUSCI_A2 modulation control register
    EUSCI_A2->MCTLW = 0x0021; //0b 0000 0000 0010 xxx1

    EUSCI_A2->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;    // Initialize eUSCI
    EUSCI_A2->IFG &= ~EUSCI_A_IFG_RXIFG;        // Clear eUSCI RX interrupt flag
    EUSCI_A2->IE |= EUSCI_A_IE_RXIE;            // Enable USCI_A2 RX interrupt

    // Enable global interrupt
    __enable_irq();

    // Enable eUSCIA2 interrupt in NVIC module
    NVIC->ISER[0] = (1 << EUSCIA2_IRQn );
}

void printMessage(const char* message, int messageLength) {
    int i;
    for (i = 0; i < messageLength; i++) {
        // Check if the TX buffer is empty first
        while(!(EUSCI_A2->IFG & EUSCI_A_IFG_TXIFG));

        // Send next character of message
        //  Note that writing to TX buffer clears the flag
        EUSCI_A2->TXBUF = message[i];
    }
}

char getLastChar() {
    return lastChar;
}

void EUSCIA2_IRQHandler(void) {
    // Check if receive flag is set (value ready in RX buffer)
    if (EUSCI_A2->IFG & EUSCI_A_IFG_RXIFG) {
        // Note that reading RX buffer clears the flag
        uint8_t digit = EUSCI_A2->RXBUF;
        // Echo character back to screen, otherwise user will not be able to
        //  verify what was typed
        while(!(EUSCI_A2->IFG & EUSCI_A_IFG_TXIFG)); // Wait for TX buffer ready
        EUSCI_A2->TXBUF = digit;                 // Echo character to terminal

        // Convert ASCII character to appropriate hexadecimal value for current digit position
        switch (digit) {
        // For numerical ASCII character, subtract 0x30 ('0') to get decimal value
        case 'p':
        case 'P':
            //PrintMessage(. . .
            lastChar = 'P';
            break;

        case 'x':
        case 'X':
            lastChar = 'X';
            break;

        case 'y':
        case 'Y':
            lastChar = 'Y';
            break;

        case 'z':
        case 'Z':
            lastChar = 'Z';
            break;

        default: // invalid hex character received
            printMessage(invalid, (sizeof(invalid)/sizeof(invalid[0])));
            break;
        }
    }
}

