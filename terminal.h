/*
 * terminal.h
 *
 *  Created on: May 1, 2023
 *      Author: teaneyje
 */

#ifndef TERMINAL_H_
#define TERMINAL_H_

#define INVALID_VALUE -1
static const char prompt[] = "\n\r(P)rint sensor values; display (X)-axis values; \n\rdisplay (Y)-axis values; display (Z)-axis values\n\rSelection: ";
static const char invalid[] = "\n\rInvalid Input";
static char lastChar = 'X';

void initializeTerminal();

char getLastChar();

void printMessage(const char* message, int messageLength);

#endif /* TERMINAL_H_ */
