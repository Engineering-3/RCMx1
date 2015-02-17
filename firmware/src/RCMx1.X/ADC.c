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
#include "system.h"
#include "user.h"
#include "ADC.h"

/******************************************************************************/
/* User Functions                                                             */
/******************************************************************************/

// Initalize the ADC.
// For this firmware, we will use 14 of the possible 16 ADC channels.
// We will set up a timer to fire an interrupt. In this interrupt code
// we'll copy the last ADC conversion out into the ADC result array and start
// the next conversion.
// ADC is set up as single ended, 12 bit right justified.
void ADCInit(void)
{
    UINT8 i;

    ADCON2bits.ADFM = 1;            // Right justified
    AnalogWriteEnable(0x7F, 0x7F);  // Set up all analogs as analog inputs, not GPIO
    // TRIS bits set in system.c
    ADCON2bits.ADCS = 0b110;        // Use FOsc/64 as TAD clock
    ADCON2bits.ACQT = 0b111;        // Use 20 Tad for acq time
    // Turn the ADC on
    ADCON0bits.ADON = 1;

    AnalogGPIOWriteDirection(0x7F, 0x7F);
    
    for (i=0; i < 14; i++)
    {
        Analog_Value[i] = 0x0000;
    }
}