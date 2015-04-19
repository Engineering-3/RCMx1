/******************************************************************************
 * RCMx1 Firmware
 * Written for Tim Jump at Engineering^3
 * By Brian Schmalz of LogicPD
 * July 2012
 * Copyright Engineering^3 2015
 *
 * This firmware runs all of the functions of the RCMx1 I/O expander board
 * The RXMx1 connects to the SRV-1 Surveyor Blackfin Camera board via I2C
 * and listens for commands from the SRV. It has analog inputs, digital I/O,
 * RC Servo outputs, and h-bridge motor driver outputs. It also breaks out all
 * of the SRV-1 bus signals.
 *
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *****************************************************************************/

/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

#include <p18f86k22.h>
#include <GenericTypeDefs.h>
#include "user.h"
#include "system.h"

/******************************************************************************/
/* Local defines
/******************************************************************************/
#define HEARTBEAT_TIMER_RELOAD  1000 // in ms

/******************************************************************************/
/* Global variable storage                                                    */
/******************************************************************************/

// Safety Timeout Value reigster
UINT8 SafetyTimeoutValue = 0;

// GPIO pins 1-6
UINT8 GPIO_IO = 0;
UINT8 GPIO_Dir = 0;
UINT8 GPIO_PullUp = 0;
UINT8 GPIO_Invert = 0;

// RC Servo pins 1-8
UINT8 RCServo_IO = 0;
UINT8 RCServo_Dir = 0;                          // Holds register 0x21 contents, used when pins are in GPIO mode
UINT8 RCServo_Invert = 0;
UINT8 RCServo_Enable[RC_SERVO_COUNT] = {0};
UINT16 RCServo_MaxWidth[RC_SERVO_COUNT] = {0};
UINT16 RCServo_MinWidth[RC_SERVO_COUNT] = {0};
UINT16 RCServo_TargetWidth[RC_SERVO_COUNT] = {0};
UINT16 RCServo_Width[RC_SERVO_COUNT] = {0};
UINT16 RCServo_MaxForward[RC_SERVO_COUNT] = {0};
UINT16 RCServo_MaxReverse[RC_SERVO_COUNT] = {0};
UINT8 RCServo_MaxAccel[RC_SERVO_COUNT] = {0};
UINT8 RCServo_MaxDecel[RC_SERVO_COUNT] = {0};
UINT8 RCServo_SlowMove[RC_SERVO_COUNT] = {0};

// Current channel number (0 through 7) that RC servo output is on
UINT8 RCServo_CurrentChannel = {0};
// Used to record, internally, if we are currently outputting a high on any RC servo pin or not
UINT8 RCServo_SignalOn = 0;
// TRUE if the 'filter' functions are enabled on this RC Servo channel
UINT8 RCServo_FilterEnabled[RC_SERVO_COUNT] = {0};

// Anaog pins 1-16
UINT8 Analog_IO[2] = {0};
UINT8 Analog_Dir[2] = {0};
UINT8 Analog_PullUp = 0;
UINT8 Analog_Invert[2] = {0};
UINT8 Analog_Enable[2] = {0};
UINT16 Analog_Value[16] = {0};

// Motor outputs 1-4
UINT8 Motor_Value[4] = {0};

// Time, in seconds, since the last read/write over I2C to this system
UINT8 LastCommandTime = 0;

// Timer used to blink the LED by the programming connector
volatile UINT16 TimerHeartbeat = 0;

/******************************************************************************/
/* User Functions                                                             */
/******************************************************************************/

/* <Initialize variables in user.h and insert code for user algorithms.> */

void AppInit(void)
{
    // Start of with heartbeat LED turned on to show that we got this far
    HEARTBEAT = 1;
    TimerHeartbeat = HEARTBEAT_TIMER_RELOAD;
}

void UserAppRun(void)
{
    // Turn off low interrupt
    INTCONbits.GIEL = 0;

    // Handle heartbeat LED
    if (TimerHeartbeat == 0)
    {
        ToggleHeartbeat();
        TimerHeartbeat = HEARTBEAT_TIMER_RELOAD;
    }

    // Turn on low interrupt
    INTCONbits.GIEL = 1;
}

void ToggleHeartbeat(void)
{
    HEARTBEAT = !HEARTBEAT;
}