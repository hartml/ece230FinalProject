#include "msp.h"

#include "rgbLED.h"
#include "servoDriver.h"
#include "stepperMotor.h"
#include "csHFXT.h""
#include "IRReceiver.h"

#define Up 0x46
#define Down 0x15
#define Left 0x44
#define Right 0x43
#define Ok 0x40

uint8_t LastButtonPressed = 0;
static const int32_t MaxStepperSpeed = 1000;
int32_t StepperSpeed = 0;
static const MaxServoAngle = 170;
static const MinServoAngle = 10;
uint8_t ServoAngle = (MaxServoAngle + MinServoAngle) / 2;

/**
 * main.c
 */
void main(void) {
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer
    RGBLED_init();
    initStepperMotor();
    enableStepperMotor();
    initServoMotor();
    configHFXT();
    InitializeIRReceiver();
    RGBLED_setColor(BLUE);

    while(1) {
        RGBLED_setColor(RED);
        setRPM(false, 1000);
        setServoAngle(90);
        static uint8_t tmpLastPressed = 0;
        tmpLastPressed = getLastPressed();
        if(tmpLastPressed != 0) {
            LastButtonPressed = tmpLastPressed;
        }
        switch(tmpLastPressed) {
        case(Up): //Increment speed unless we're already at max speed
            StepperSpeed += 100; //we pass in 100x rpm, so this isn't one.
            if(StepperSpeed > MaxStepperSpeed) {
                StepperSpeed = MaxStepperSpeed;
            }
            break;
        case(Down): //Decrement speed unless we're already stopped or all the way reverse
            StepperSpeed -= 100; //we pass in 100x rpm, so this isn't one.
            if(StepperSpeed < -1 * MaxStepperSpeed) {
                StepperSpeed = -1 * MaxStepperSpeed;
            }
            break;
        case(Left): //Make servo turn left (unless already at max left turn)
            ServoAngle -= 10;
            if(ServoAngle < MinServoAngle) {
                ServoAngle = MinServoAngle;
            }
            break;
        case(Right): //Make servo turn right (unless already at max right turn)
            ServoAngle -= 10;
            if(ServoAngle < MinServoAngle) {
                ServoAngle = MinServoAngle;
            }
            break;
        case(Ok): //Reset the stuff so that speed = 0 and turn is at center.
            ServoAngle = (MaxServoAngle + MinServoAngle) / 2;
            StepperSpeed = 0;
            break;
        default:
            //Do Nothing. This is just a placeholder for rn
            break;
        }
    } //End Main loop
} //End main
