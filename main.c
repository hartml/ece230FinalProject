#include "msp.h"
#include "lcd.h"
#include "terminal.h"
#include "csHFXT.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

//START Terminal Variables/Defines

//STOP Terminal Variables/Defines
//START Gyro Variables/Defines
#define NUM_OF_REC_BYTES        6       // number of bytes to receive from sensor read
#define GY521_ADDRESS           0x68    // I2C address of GY-521 sensor
#define ACCEL_BASE_ADDR         0x3B    // base address of accelerometer data registers
#define PWR_MGMT_ADDR           0x6B    // address of power management register
#define GyroConstant 15 //TODO: Find this value

uint8_t RXData[NUM_OF_REC_BYTES] = {0, 0, 0, 0, 0, 0};
uint8_t RXDataPointer, TXDataPointer;
int16_t Ax, Ay, Az = 0;
int32_t Gx1000x, Gy1000x, Gz1000x = 0;

void initializeGyro();
void updateGyro();
//STOP Gyro Variables/Defines
//START LCD Variables/Defines
uint8_t LCDtopMessageLength = 17;
char LCDtopMessage[17] = " Accelerometer 8g";
uint16_t LCDbottomMessageLength = 17;
char LCDbottomMessage[17] = " X:  X.XXX g     ";
uint16_t TerminalPMessageLength = 45;
char TerminalPMessage[52] = "Accel_X: X.XXX g Accel_Y: -X.XXX g Accel_Z: X.XXX g";

void updatePMessage();
void updateBottomMessage(int32_t GX1000x, char Axis);
//STOP LCD Variables/Defines

void main(void) {
    volatile int16_t accel_x, accel_y, accel_z;
    volatile uint32_t i;
    configHFXT();
    initializeGyro();
    configLCD(48000000);
    initLCD();
    initializeTerminal();
    /* Stop Watchdog timer */
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;

    while (1) {
        updateGyro();
        printMessage(prompt, (sizeof(prompt)/sizeof(prompt[0])));
        lastChar = 'Z';
        switch(lastChar) {
        case('P'):
            updatePMessage();
            printMessage(TerminalPMessage, TerminalPMessageLength);
            break;
        case('X'):
            updateBottomMessage(Gx1000x, 'X');
            writeSentence(LCDbottomMessage, LCDbottomMessageLength, 1);
            break;
        case('Y'):
            updateBottomMessage(Gy1000x, 'Y');
            writeSentence(LCDbottomMessage, LCDbottomMessageLength, 1);
            break;
        case('Z'):
            updateBottomMessage(Gz1000x, 'Z');
            writeSentence(LCDbottomMessage, LCDbottomMessageLength, 1);
            break;
        } //end switch
        writeSentence(LCDtopMessage, LCDtopMessageLength, 0); //TODO: Make this and next line happen at 1Hz
        writeSentence(LCDbottomMessage, LCDbottomMessageLength, 1);
    } //end main loop
} //end main

//START LCD Methods
void updateBottomMessage(int32_t GX1000x, char Axis) {
    if(GX1000x < 0) {
        LCDbottomMessage[4] = '-';
        GX1000x = -1 * GX1000x;
    } else {
        LCDbottomMessage[4] = ' ';
    }
    char ones = GX1000x % 10 + '0';
    GX1000x /= 10;
    char tens = GX1000x % 10 + '0';
    GX1000x /= 10;
    char hundreds = GX1000x % 10 + '0';
    GX1000x /= 10;
    char thousands = GX1000x % 10 + '0';

    LCDbottomMessage[1] = Axis;
    LCDbottomMessage[5] = thousands;
    LCDbottomMessage[7] = hundreds;
    LCDbottomMessage[8] = tens;
    LCDbottomMessage[9] = ones;
}

void updatePMessage() {
    char Axones = Ax % 10 + '0';
    Ax /= 10;
    char Axtens = Ax % 10 + '0';
    Ax /= 10;
    char Axhundreds = Ax % 10 + '0';
    Ax /= 10;
    char Axthousands = Ax % 10 + '0';
    Ax /= 10;

    char Ayones = Ay % 10 + '0';
    Ay /= 10;
    char Aytens = Ay % 10 + '0';
    Ay /= 10;
    char Ayhundreds = Ay % 10 + '0';
    Ax /= 10;
    char Aythousands = Ay % 10 + '0';
    Ay /= 10;

    char Azones = Az % 10 + '0';
    Az /= 10;
    char Aztens = Az % 10 + '0';
    Ax /= 10;
    char Azhundreds = Az % 10 + '0';
    Az /= 10;
    char Azthousands = Az % 10 + '0';
    Az /= 10;

    TerminalPMessage[10] = Axthousands;
    TerminalPMessage[12] = Axhundreds;
    TerminalPMessage[13] = Axtens;
    TerminalPMessage[14] = Axones;

    TerminalPMessage[29] = Aythousands;
    TerminalPMessage[31] = Ayhundreds;
    TerminalPMessage[32] = Aytens;
    TerminalPMessage[33] = Ayones;

    TerminalPMessage[38] = Azthousands;
    TerminalPMessage[40] = Azhundreds;
    TerminalPMessage[41] = Aztens;
    TerminalPMessage[42] = Azones;
}

//STOP LCD Methods
//START Gyro Methods
// I2C interrupt service routine
void EUSCIB0_IRQHandler(void) {
    // Handle if ACK not received for address frame
    if (EUSCI_B0->IFG & EUSCI_B_IFG_NACKIFG) {
        EUSCI_B0->IFG &= ~ EUSCI_B_IFG_NACKIFG;

        // resend I2C start condition and address frame
        EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTT;
        TXDataPointer = 0;
        RXDataPointer = 0;
    }
    // When TX buffer is ready, load next byte or Restart for Read
    if (EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0) {
        if (TXDataPointer == 0) {
            // load 1st data byte into TX buffer (writing to buffer clears the flag)
            EUSCI_B0->TXBUF = ACCEL_BASE_ADDR;      // send register address
            TXDataPointer = 1;
        } else {
            // change to receiver mode (Read)
            EUSCI_B0->CTLW0 &= ~EUSCI_B_CTLW0_TR;
            // send Restart and address frame with R bit
            EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTT;
            TXDataPointer = 0;
            RXDataPointer = 0;
            // need to clear flag since not writing to buffer
            EUSCI_B0->IFG &= ~ EUSCI_B_IFG_TXIFG0;
        }
    }
    // When new byte is received, read value from RX buffer
    if (EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG0) {
        // Get RX data
        if (RXDataPointer < NUM_OF_REC_BYTES) {
            // reading the buffer clears the flag
            RXData[RXDataPointer++] = EUSCI_B0->RXBUF;
        }
        else {  // in case of glitch, avoid array out-of-bounds error
            EUSCI_B0->IFG &= ~ EUSCI_B_IFG_RXIFG0;
        }

        // check if last byte being received - if so, initiate STOP (and NACK)
        if (RXDataPointer == (NUM_OF_REC_BYTES-1)) {
            // Send I2C stop condition
            EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
        }
    }
}

void updateGyro() {
    // Ensure stop condition got sent
    while (EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTP);

    /* Read register values from sensor by sending register address and restart
     *
     *  format for Write-Restart-Read operation
     *  _______________________________________________________________________
     *  |       | Periph | <Register |       | Periph |               |       |
     *  | Start |  Addr  |  Address> | Start |  Addr  | <6 Byte Read> | Stop  |
     *  |_______|____W___|___________|_______|____R___|_______________|_______|
     *
     *
     *  Initiated with start condition - completion handled in ISR
     */
    // change to transmitter mode (Write)
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TR;
    // send I2C start condition with address frame and W bit
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTT;

    // wait for sensor data to be received
    while (RXDataPointer < NUM_OF_REC_BYTES) ;
    /* DONE combine bytes to form 16-bit accel_ values  */
    Ax = RXData[1] + (uint16_t) (RXData[0] << 8);
    Ay = RXData[3] + (uint16_t) (RXData[2] << 8);
    Az = RXData[5] + (uint16_t) (RXData[4] << 8);
    Gx1000x = Ax / GyroConstant;
    Gy1000x = Ay / GyroConstant;
    Gz1000x = Az / GyroConstant;

    RXDataPointer = 0;
}

void initializeGyro() {
    /* Configure UART pins */
    P1->SEL0 |= BIT6 | BIT7;                // set I2C pins as secondary function
    P1->SEL1 &= ~(BIT6 | BIT7);

    // Initialize data variable
    RXDataPointer = 0;
    TXDataPointer = 0;

    /* Configure eUSCI_B0 for I2C mode
     *  I2C master mode, synchronous, 7-bit address, SMCLK clock source,
     *  transmit mode, with automatic STOP condition generation
     */
    EUSCI_B0->CTLW0 |= EUSCI_A_CTLW0_SWRST; // Software reset enabled
    EUSCI_B0->CTLW0 = EUSCI_A_CTLW0_SWRST | // Remain eUSCI in reset mode
            EUSCI_B_CTLW0_MODE_3 |          // I2C mode
            EUSCI_B_CTLW0_MST |             // Master mode
            EUSCI_B_CTLW0_SYNC |            // Sync mode
            EUSCI_B_CTLW0_TR |              // Transmitter mode
            EUSCI_B_CTLW0_SSEL__SMCLK;      // SMCLK

    /* I2C clock calculation
     * Refer to Section 26.3.6 of Technical Reference manual
     * BRCLK = 3MHz, I2C bit clock rate = 100kbps
    */
    // DONE configure eUSCI_B0 bit rate control for 100 kbps
    EUSCI_B0->BRW = 30; //Giving a prescalar of 1:30, so a 100kb tick/s

    /* Configure I2C to communicate with GY-521 */
    EUSCI_B0->I2CSA = GY521_ADDRESS;            // I2C peripheral address
    EUSCI_B0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;    // Release eUSCI from reset

    /* Initialize GY-521 by writing to Power Management Register
     *
     *  format for Write operations
     *  _________________________________________________________________
     *  |       |          |                 |                  |       |
     *  | Start |  Addr  W | <Register Addr> | <Value to write> | Stop  |
     *  |_______|__________|_________________|__________________|_______|
     */
    // Ensure stop condition not pending
    while (EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTP);
    do {
        // Send I2C start condition and address frame with W
        EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTT;
        // wait for TX buffer to be ready
        while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
        // load 1st data byte into TX buffer
        EUSCI_B0->TXBUF = PWR_MGMT_ADDR;            // send register address
        // wait for ACK/NACK after address frame
        while (EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTT);
    } while(EUSCI_B0->IFG & EUSCI_B_IFG_NACKIFG);   // resend address frame if ACK not received
    // wait for TX buffer to be ready
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
    // load 2nd data byte into TX buffer
    EUSCI_B0->TXBUF = 0;                // write value to register
    // wait for 2nd data byte to begin transmit
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
    // Send I2C stop condition
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;

    // Ensure stop condition got sent
    while (EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTP);
    // ensure flags are cleared before enabling interrupts
    EUSCI_B0->IFG &= ~(EUSCI_B_IFG_TXIFG0 | EUSCI_B_IFG_RXIFG0 | EUSCI_B_IFG_NACKIFG);

    EUSCI_B0->IE |= EUSCI_A_IE_RXIE |       // Enable receive interrupt
            EUSCI_A_IE_TXIE |               // Enable transmit interrupt
            EUSCI_B_IE_NACKIE;              // Enable NACK interrupt

    // Enable eUSCIB0 interrupt in NVIC module
    NVIC->ISER[0] = (1 << EUSCIB0_IRQn);

    // Enable global interrupt
    __enable_irq();
}
//STOP Gyro Methods
