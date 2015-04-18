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
/* User Level #define Macros                                                  */
/******************************************************************************/
#include <GenericTypeDefs.h>
#include "system.h"

/******************************************************************************/
/* Externs for public variables                                               */
/******************************************************************************/
// GPIO pins 1-16
extern UINT8 GPIO_IO;
extern UINT8 GPIO_Dir;
extern UINT8 GPIO_PullUp;
extern UINT8 GPIO_Invert;

// RC Servo pins 1-8
extern UINT8 RCServo_IO;
extern UINT8 RCServo_Dir;
extern UINT8 RCServo_Invert;
extern UINT8 RCServo_Enable[RC_SERVO_COUNT];
extern UINT16 RCServo_MaxWidth[RC_SERVO_COUNT];
extern UINT16 RCServo_MinWidth[RC_SERVO_COUNT];
extern UINT16 RCServo_TargetWidth[RC_SERVO_COUNT];
extern UINT16 RCServo_Width[RC_SERVO_COUNT];
extern UINT16 RCServo_MaxForward[RC_SERVO_COUNT];
extern UINT16 RCServo_MaxReverse[RC_SERVO_COUNT];
extern UINT8 RCServo_MaxAccel[RC_SERVO_COUNT];
extern UINT8 RCServo_MaxDecel[RC_SERVO_COUNT];
extern UINT8 RCServo_SafetyTimeout[RC_SERVO_COUNT];
extern UINT8 RCServo_SlowMove[RC_SERVO_COUNT];
extern UINT8 RCServo_CurrentChannel;
extern UINT8 RCServo_SignalOn;
extern UINT8 RCServo_FilterLastCommandTime[RC_SERVO_COUNT];
extern UINT8 RCServo_FilterEnabled[RC_SERVO_COUNT];

// Anaog pins 1-16
extern UINT8 Analog_IO[2];
extern UINT8 Analog_Dir[2];
extern UINT8 Analog_PullUp;
extern UINT8 Analog_Invert[2];
extern UINT8 Analog_Enable[2];
extern UINT16 Analog_Value[16];

// Motor outputs 1-4
extern UINT8 Motor_Value[4];
extern UINT8 Motor_Safety_Timeout[4];

// Countdown timer for heartbeat LED
extern volatile UINT16 TimerHeartbeat;

#define HEARTBEAT   LATAbits.LATA4

/******************************************************************************/
/* User Function Prototypes                                                   */
/******************************************************************************/

/* TODO User level functions prototypes (i.e. InitApp) go here */

void AppInit(void);         /* I/O and Peripheral Initialization */
void UserAppRun(void);      // Run normal main-loop app code
void ToggleHeartbeat(void); // Togggle heartbeat LED
