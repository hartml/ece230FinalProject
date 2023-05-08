#include "msp.h"
#include "servoDriver.h"
#include "stepperMotor.h"
#include "sysTickDelays.h"
#include "csHFXT.h"
#include <stdint.h>
#include <stdio.h>


void main(void) {
    initServoMotor();      // initialize servo
    initStepperMotor();    // initialize stepper
    enableStepperMotor();  // enable stepper
}
    while (1) {

   } //end main
