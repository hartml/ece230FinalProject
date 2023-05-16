#include "msp.h"

#include "rgbLED.h"
#include "servoDriver.h"
#include "stepperMotor.h"
#include "csHFXT.h""
#include "IRReceiver.h"

uint8_t LastButtonPressed = 0;

/**
 * main.c
 */
void main(void) {
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer
	RGBLED_init();
	initStepperMotor();
	enableStepperMotor();
	initServoMotor();
	configHFXT();
	InitializeIRReceiver();

	while(1) {
	    RGBLED_setColor(BLUE);
	    setRPM(false, 1000);
	    setServoAngle(10);
	    static uint8_t tmpLastPressed = 0;
	    tmpLastPressed = getLastPressed();
	    if(tmpLastPressed != 0) {
	        LastButtonPressed = tmpLastPressed;
	    }
	} //End Main loop
} //End main
