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
#include "PWM.h"

/******************************************************************************/
/* User Functions                                                             */
/******************************************************************************/

// We will use PWMs 2,4,5,6,7,8,9,10 for the LEGO motor H-bridges
// Each of the 4 motors will be controlled by a pair of PWM outputs
// Only one of the pair will be on at a time.
// All will use Timer2 as a timebase.
void PWMInit(void)
{
    // Select TMR2 for all PWM modules
    CCPTMRS0bits.C2TSEL = 0b000;
    CCPTMRS1bits.C7TSEL = 0b00;
    CCPTMRS1bits.C6TSEL0 = 0b0;
    CCPTMRS1bits.C5TSEL0 = 0b0;
    CCPTMRS1bits.C4TSEL = 0b00;
    CCPTMRS2bits.C10TSEL0 = 0b0;
    CCPTMRS2bits.C9TSEL0 = 0b0;
    CCPTMRS2bits.C8TSEL = 0b00;

    // Set all duty cycle values to zero (off)
    CCPR2L = 0x00;
    CCPR4L = 0x00;
    CCPR5L = 0x00;
    CCPR6L = 0x00;
    CCPR7L = 0x00;
    CCPR8L = 0x00;
    CCPR9L = 0x00;
    CCPR10L = 0x00;
    
    // Set PWM as the mode for all modules
    CCP2CONbits.CCP2M = 0b1100;
    CCP4CONbits.CCP4M = 0b1100;
    CCP5CONbits.CCP5M = 0b1100;
    CCP6CONbits.CCP6M = 0b1100;
    CCP7CONbits.CCP7M = 0b1100;
    CCP8CONbits.CCP8M = 0b1100;
    CCP9CONbits.CCP9M = 0b1100;
    CCP10CONbits.CCP10M = 0b1100;
}

// Motor channels are from 0 to 3, and indicate which h-bridge to control
// NewValue is the new speed value. 0x00 = off, 0x80 = break, 0x01 full reverse
// 0xFF = full forward.
void PWMUpdateValue(UINT8 MotorChannel, UINT8 NewValue)
{
    switch (MotorChannel)
    {
    // PWM4,5 - Motor output B, on LEGO_MOTOR_4A and LEGO_MOTOR_4B
    case 1:
        // Stop/coast
        if (NewValue == 0x00)
        {
            CCPR4L = 0x00;
            CCPR5L = 0x00;
        }
        // Stop/Break
        else if (NewValue == 0x80)
        {
            CCPR4L = 0xFF;
            CCPR5L = 0xFF;
        }
        // Forward
        else if (NewValue > 0x80)
        {
            CCPR4L = (NewValue - 0x80) << 1;
            CCPR5L = 0x00;
        }
        // Reverse
        else
        {
            CCPR4L = 0x00;
            CCPR5L = (0x80 - NewValue) << 1;
        }
        break;

    // PWM2,6 : Motor output A, on LEGO_MOTOR_3A and LEGO_MOTOR_3B
    case 0:
        // Stop/coast
        if (NewValue == 0x00)
        {
            CCPR6L = 0x00;
            CCPR2L = 0x00;
        }
        // Stop/Break
        else if (NewValue == 0x80)
        {
            CCPR6L = 0xFF;
            CCPR2L = 0xFF;
        }
        // Forward
        else if (NewValue > 0x80)
        {
            CCPR6L = (NewValue - 0x80) << 1;
            CCPR2L = 0x00;
        }
        // Reverse
        else
        {
            CCPR6L = 0x00;
            CCPR2L = (0x80 - NewValue) << 1;
        }
        break;

    // PWM7,8 : Motor output D, on LEGO_MOTOR_2A and LEGO_MOTOR_2B
    case 3:
        // Stop/coast
        if (NewValue == 0x00)
        {
            CCPR7L = 0x00;
            CCPR8L = 0x00;
        }
        // Stop/Break
        else if (NewValue == 0x80)
        {
            CCPR7L = 0xFF;
            CCPR8L = 0xFF;
        }
        // Forward
        else if (NewValue > 0x80)
        {
            CCPR7L = (NewValue - 0x80) << 1;
            CCPR8L = 0x00;
        }
        // Reverse
        else
        {
            CCPR7L = 0x00;
            CCPR8L = (0x80 - NewValue) << 1;
        }
        break;

    // PWM9,10 : Motor output C, on LEGO_MOTOR_1A and LEGO_MOTOR_1B
    case 2:
        // Stop/coast
        if (NewValue == 0x00)
        {
            CCPR9L = 0x00;
            CCPR10L = 0x00;
        }
        // Stop/Break
        else if (NewValue == 0x80)
        {
            CCPR9L = 0xFF;
            CCPR10L = 0xFF;
        }
        // Forward
        else if (NewValue > 0x80)
        {
            CCPR9L = (NewValue - 0x80) << 1;
            CCPR10L = 0x00;
        }
        // Reverse
        else
        {
            CCPR9L = 0x00;
            CCPR10L = (0x80 - NewValue) << 1;
        }
        break;

    default:
        break;
    }
}
