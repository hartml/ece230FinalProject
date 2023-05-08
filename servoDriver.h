/*! \file */
/*!
 * servoDriver.h
 *
 * Description: Servo motor driver for MSP432P4111 Launchpad.
 *              Assumes SMCLK configured with 48MHz HFXT as source.
 *              Uses Timer_A2 and P5.6 (TA2.1)
 *
 *  Created on:
 *      Author:
 */

#ifndef SERVODRIVER_H_
#define SERVODRIVER_H_

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

#include "msp.h"

// DONE add tick count values for constants
#define SERVO_TMR_PERIOD                60000        // ticks for 40Hz signal
#define SERVO_MIN_ANGLE                 1800        // ticks for 0.75ms pulse
#define SERVO_MAX_ANGLE                 5400        // ticks for 2.25ms pulse
#define TEN_DEGREE_TICKS                200        // ticks 10 degree shift
#define ONE_DEGREE_TICKS                20         //ticks 1 degree shift


/*!
 * \brief This function configures pins and timer for servo motor driver
 *
 * This function configures P5.6 as output pin for servo drive signal and
 *  initializes Timer_A2 CCR1 for PWM output
 *
 * Modified bit 6 of \b P2DIR register and \b P2SEL registers.
 * Modified \b TA2CTL, \b TA2CCTL1 and CCR registers.
 *
 * \return None
 */
extern void initServoMotor(void);


/*!
 * \brief This increments servo angle 10 degrees, with wrap-around
 *
 * This function increments servo angle by 10 degrees. If new angle exceeds max
 *  angle (+90), it wraps around to min angle (-90)
 *
 * Modified \b TA2CCR1 register.
 *
 * \return None
 */
extern void incrementTenDegree(void);


/*!
 * \brief This function sets angle of servo
 *
 * This function sets angle of servo to \a angle (between -90 to 90)
 *
 *  \param angle Angle in degrees to set servo (between -90 to 90)
 *
 * Modified \b TA2CCR1 register.
 *
 * \return None
 */
extern void setServoAngle(uint8_t angle);


//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif /* SERVODRIVER_H_ */
