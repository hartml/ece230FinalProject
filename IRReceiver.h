/*
 * IRReceiver.h
 *
 *  Created on: May 15, 2023
 *      Author: teaneyje
 */

#ifndef IRRECEIVER_H_
#define IRRECEIVER_H_

/**
 * Initializes Timer_A0 and it's CCR modules (0 and 1) to be able to read the input from the IR receiver hardware
 */
extern void InitializeIRReceiver();

/**
 * Returns the last pressed button, returns '\0' if already called before the next button was pressed or no character to return
 */
extern char getLastPressed();

#endif /* IRRECEIVER_H_ */
