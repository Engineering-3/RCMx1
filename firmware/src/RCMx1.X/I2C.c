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
#include "user.h"
#include "i2c.h"
#include "system.h"

/******************************************************************************/
/* User Functions                                                             */
/******************************************************************************/

/* <Initialize variables in user.h and insert code for user algorithms.> */

void I2CInit(void)
{
    // initalize I/O pins as inputs
    TRISDbits.TRISD5 = INPUT;
    TRISDbits.TRISD6 = INPUT;

    // Set up I2C control registers
    SSP2CON1bits.SSPM = 0b0110; // Slave mode, 7-bit addressing

    // Set up our slave address
    SSP2ADD = RCMX1_I2C_ADDRESS;

    // clear the interrupt bit if set
    PIR2bits.SSP2IF = 0;
    // Turn on the I2C interrupt
    PIE2bits.SSP2IE = 1;
    // And set it to low priority
    IPR2bits.SSP2IP = 0;
    
    // Turn the module on
    SSP2CON1bits.SSPEN = 1;
}

