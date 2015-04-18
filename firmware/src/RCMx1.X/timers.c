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
#include "timers.h"

/******************************************************************************/
/* Local variables
/******************************************************************************/

/******************************************************************************/
/* User Functions                                                             */
/******************************************************************************/

// Timer0: Unused
// Timer1: Unused
// Timer2: Timebase for all motor PWMs
// Timer3: Used for software based RC servo timing (high ISR)
// Timer4: 1ms ISR for ADC conversion and general timing (low ISR)
// Timer5-10: Unused
void TimerInit(void)
{
    UINT8 i;
    
    // Timer2 - set up as timebase for all PWMs
    // With 1:1 presacle and 1:1 postscale, timer couts at 16MHz.
    // With period register (PR2) set to 256, PWM will be at 62.5KHz.
    T2CONbits.T2OUTPS = 0b0000;     // 1:1 postscale
    T2CONbits.T2CKPS = 0b00;        // 1:1 prescale
    TMR2 = 0;                       // Clear timer
    PR2 = 0xFE;                     // Set period
    T2CONbits.TMR2ON = 1;           // Turn on the timer

    // Timer3 - Normally set up to fire every 24ms. When RC servos active,
    //      this timer fires the high ISR that will set/clear the RC servo pins
    // We need to set a value of 0xFFFF - 48,000 in the TMR3 register so that
    // it overflows in 48,000 ticks, giving us a period of 24ms.
    // Tick rate is 16Mhz/8 or 2MHz.
    T3CONbits.TMR3CS = 0b00;        // Fosc/4 as clock source
    T3CONbits.T3CKPS = 0b11;        // 1:8 prescale
    T3CONbits.RD16 = 1;             // Enable read/write in 16-bit mdoe
    TMR3H = (0xFFFF - RC_SERVO_FRAME_TIME) >> 8;       // Set to inital value
    TMR3L = (0xFFFF - RC_SERVO_FRAME_TIME) & 0xFF;     //
    PIR2bits.TMR3IF = 0;            // Clear the interrupt flag
    PIE2bits.TMR3IE = 1;            // Enable the timer to generate interrupts
    IPR2bits.TMR3IP = 1;            // And set it to high priority
    T3CONbits.TMR3ON = 1;           // Turn the timer on

    // Timer4 - Triggers 1ms low ISR
    // Clocked at Fosc/4, with 1:16 prescale and PR = 240,
    // the timer will overflow every 250uS. With a postscale
    // of 1:4, the ISR will fire every 1ms.
    T4CONbits.T4OUTPS = 0b0011;     // 1:4 postscale
    T4CONbits.T4CKPS = 0b11;        // 1:16 prescale
    PR4 = 250;                      // Set match register to 250
    TMR4 = 0;                       // Clear counter to zero
    PIR5bits.TMR4IF = 0;            // Clear the interrupt flag
    PIE5bits.TMR4IE = 1;            // Enable the timer to generate interrupts
    IPR5bits.TMR4IP = 0;            // And set it to low priority
    T4CONbits.TMR4ON = 1;           // Turn the timer on

    // Initalize all of the RC Servo variables
    for (i = 0; i < RC_SERVO_COUNT; i++)
    {
        RCServo_Enable[i] = RC_SERVO_ENABLE_ON;
        RCServo_MaxWidth[i] = TMR3_MS_TO_TICKS(2);
        RCServo_MinWidth[i] = TMR3_MS_TO_TICKS(1);
        RCServo_TargetWidth[i] = RCServo_Scale(0x80, i);

        RCServo_MaxForward[i] = RCServo_Scale(0xFF, i);
        RCServo_MaxReverse[i] = RCServo_Scale(0x01, i);
        RCServo_MaxAccel[i] = RCServo_AccelScale(0xFF);
        RCServo_MaxDecel[i] = RCServo_AccelScale(0xFF);
        RCServo_SafetyTimeout[i] = DEFAULT_SAFETY_TIMEOUT_S;
        RCServo_Width[i] = RCServo_Scale(0x80, i);
        RCServo_FilterLastCommandTime[i] = 0;
        RCServo_SlowMove[i] = 0x00;
        RCServo_FilterEnabled[i] = FALSE;
    }
    RCServo_CurrentChannel = 0;
    RCServo_SignalOn = FALSE;
}

// Take a 0x00 to 0xFF position from the SRV and convert it to a RCServo_Width[] value
// to be used by the timer hardware on the ARM. Scaled by MIN and MAX. Each
// Channel can have it's own scaling.
// Note we have a 'fudge factor' of 2 added in so that the resulting UnScale
// value always matches what we send in to Scale
UINT16 RCServo_Scale(UINT8 InputVal, UINT8 Channel)
{
    return (
		RCServo_MinWidth[Channel]
        +
        (
            (
                (UINT32)(
                    RCServo_MaxWidth[Channel]
                    -
                    RCServo_MinWidth[Channel]
                )
                *
                InputVal
            )
            /
            0x100
        )
        +
        2
    );
}

// Does the inverse of RCServo_Scale(). Takes a width in timer ticks and returns
// a scaled 0x01 to 0xFF value. Needs Channel number to do scaling right.
UINT8 RCServo_Unscale(UINT16 OrigRCServo_Width, UINT8 Channel)
{
    if (OrigRCServo_Width <= RCServo_MinWidth[Channel])
    {
        return (0x00);
    }
    if (OrigRCServo_Width >= RCServo_MaxWidth[Channel])
    {
        return (0xFF);
    }
    return (
        (
            (
                OrigRCServo_Width
                -
                RCServo_MinWidth[Channel]
            )
            *
            (UINT32)0x100
        )
        /
        (
            RCServo_MaxWidth[Channel]
            -
            RCServo_MinWidth[Channel]
        )
    );
}

UINT16 RCServo_AccelScale(UINT8 InputVal)
{
//    return (InputVal << 4);
    return (InputVal);
}

UINT8 RCServo_AccelUnScale(UINT16 InputVal)
{
//    return (InputVal >> 4);
    return (InputVal);
}
