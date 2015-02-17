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
/*Files to Include                                                            */
/******************************************************************************/

#include <p18f86k22.h>
#include <GenericTypeDefs.h>
#include "system.h"
#include "user.h"

// List the actual analog input channel numbers in pin order (from pin 1
// through pin 14)
const UINT8 AnalogPinMap[14] = {19, 5, 10, 8, 6, 1, 3, 18, 11, 9, 7, 0, 2, 4};

/* Config bit settings */
#pragma config RETEN = OFF
#pragma config INTOSCSEL = LOW
#pragma config SOSCSEL = DIG
#pragma config XINST = OFF
#pragma config FOSC = INTIO2
#pragma config PLLCFG = ON
#pragma config FCMEN = OFF
#pragma config IESO = ON
#pragma config PWRTEN = OFF
#pragma config BOREN = ON
#pragma config BORV = 0
#pragma config BORPWR = LOW
#pragma config WDTEN = OFF
#pragma config WDTPS = 1
#pragma config RTCOSC = INTOSCREF
#pragma config CCP2MX = PORTBE
#pragma config MSSPMSK = MSK7
#pragma config MCLRE = ON
#pragma config STVREN = OFF
#pragma config BBSIZ = BB1K
#pragma config DEBUG = ON
#pragma config CP0 = OFF
#pragma config CP1 = OFF
#pragma config CP2 = OFF
#pragma config CP3 = OFF
#pragma config CPB = OFF
#pragma config CPD = OFF
#pragma config WRT0 = OFF
#pragma config WRT1 = OFF
#pragma config WRT2 = OFF
#pragma config WRT3 = OFF
#pragma config WRTC = OFF
#pragma config WRTB = OFF
#pragma config WRTD = OFF
#pragma config EBRT0 = OFF
#pragma config EBRT1 = OFF
#pragma config EBRT2 = OFF
#pragma config EBRT3 = OFF
#pragma config EBRTB = OFF


/* Refer to the device datasheet for information about available
oscillator configurations. */
void ConfigureOscillator(void)
{
    // Use 16Mhz from internal oscillator
    OSCCONbits.IRCF = 0b111;
    
    // Turn on the PLL
    OSCTUNEbits.PLLEN = 1;
}

void IOInit(void)
{
    // Set up TRIS bits for entire system
    // See RCMx1 PinTable.xls
    TRISA = 0x2F;
    TRISB = 0xFF;
    TRISC = 0xC0;       // 0b11000000
    TRISD = 0x6F;
    TRISE = 0x03;
    TRISF = 0xFF;
    TRISG = 0x27;

    AnalogGPIOWriteDirection(0x7F, 0x7F);

    INTCON2bits.RBPU = GPIO_PULLUP_DISABLE;
    GPIO_PullUp = 0;
    
    // For debug
    DEBUG1_TRIS = OUTPUT;
    DEBUG2_TRIS = OUTPUT;
    DEBUG3_TRIS = OUTPUT;
    DEBUG4_TRIS = OUTPUT;
    DEBUG5_TRIS = OUTPUT;
    DEBUG6_TRIS = OUTPUT;
    DEBUG7_TRIS = OUTPUT;
    DEBUG8_TRIS = OUTPUT;

    DEBUG1 = 0;
    DEBUG2 = 0;
    DEBUG3 = 0;
    DEBUG4 = 0;
    DEBUG5 = 0;
    DEBUG6 = 0;
    DEBUG7 = 0;
    DEBUG8 = 0;
}

// Take an internal representation register of the GPIO pins,
// and convert to the actual bit positions for outputting to a port
UINT8 GPIOPinMapOut(UINT8 Input)
{
    UINT8 Result = 0;

    if (Input & 0x01)
    {
        Result |= GPIO1_BIT;
    }
    if (Input & 0x02)
    {
        Result |= GPIO2_BIT;
    }
    if (Input & 0x04)
    {
        Result |= GPIO3_BIT;
    }
    if (Input & 0x08)
    {
        Result |= GPIO4_BIT;
    }
    if (Input & 0x10)
    {
        Result |= GPIO5_BIT;
    }
    if (Input & 0x20)
    {
        Result |= GPIO6_BIT;
    }
    return Result;
}

// Reading from GPIO pins, mapping to internal register bits
UINT8 GPIOPinMapIn(UINT8 Input)
{
    UINT8 Result = 0;

    if (Input & GPIO1_BIT)
    {
        Result |= 0x01;
    }
    if (Input & GPIO2_BIT)
    {
        Result |= 0x02;
    }
    if (Input & GPIO3_BIT)
    {
        Result |= 0x04;
    }
    if (Input & GPIO4_BIT)
    {
        Result |= 0x08;
    }
    if (Input & GPIO5_BIT)
    {
        Result |= 0x10;
    }
    if (Input & GPIO6_BIT)
    {
        Result |= 0x20;
    }
    return Result;
}

// Modify Input by GPIO invert bits and return value
UINT8 GPIOInvert(UINT8 Input)
{
    return (Input ^ GPIO_Invert);
}

// Read the TRIS bits for RCServo bits
UINT8 RCServoGPIOReadDirection(void)
{
    UINT8 Result = 0;
    if (RCSERVO1_TRIS == 1)
    {
        Result = Result | 0x01;
    }
    if (RCSERVO2_TRIS == 1)
    {
        Result = Result | 0x02;
    }
    if (RCSERVO3_TRIS == 1)
    {
        Result = Result | 0x04;
    }
    if (RCSERVO4_TRIS == 1)
    {
        Result = Result | 0x08;
    }
    if (RCSERVO5_TRIS == 1)
    {
        Result = Result | 0x10;
    }
    if (RCSERVO6_TRIS == 1)
    {
        Result = Result | 0x20;
    }
    if (RCSERVO7_TRIS == 1)
    {
        Result = Result | 0x40;
    }
    if (RCSERVO8_TRIS == 1)
    {
        Result = Result | 0x80;
    }

    return Result;
}

// Write the TRIS bits for RCServo bits
void RCServoGPIOWriteDirection(UINT8 Input)
{
    if (Input & 0x01)
    {
        RCSERVO1_TRIS = 1;
    }
    else
    {
        RCSERVO1_TRIS = 0;
    }
    if (Input & 0x02)
    {
        RCSERVO2_TRIS = 1;
    }
    else
    {
        RCSERVO2_TRIS = 0;
    }
    if (Input & 0x04)
    {
        RCSERVO3_TRIS = 1;
    }
    else
    {
        RCSERVO3_TRIS = 0;
    }
    if (Input & 0x08)
    {
        RCSERVO4_TRIS = 1;
    }
    else
    {
        RCSERVO4_TRIS = 0;
    }
    if (Input & 0x10)
    {
        RCSERVO5_TRIS = 1;
    }
    else
    {
        RCSERVO5_TRIS = 0;
    }
    if (Input & 0x20)
    {
        RCSERVO6_TRIS = 1;
    }
    else
    {
        RCSERVO6_TRIS = 0;
    }
    if (Input & 0x40)
    {
        RCSERVO7_TRIS = 1;
    }
    else
    {
        RCSERVO7_TRIS = 0;
    }
    if (Input & 0x80)
    {
        RCSERVO8_TRIS = 1;
    }
    else
    {
        RCSERVO8_TRIS = 0;
    }
}

// Write a byte out to RCServo pins as GPIOs
void RCServoGPIOWrite(UINT8 Input)
{
    if (Input & 0x01)
    {
        RCSERVO1_PIN = 1;
    }
    else
    {
        RCSERVO1_PIN = 0;
    }
    if (Input & 0x02)
    {
        RCSERVO2_PIN = 1;
    }
    else
    {
        RCSERVO2_PIN = 0;
    }
    if (Input & 0x04)
    {
        RCSERVO3_PIN = 1;
    }
    else
    {
        RCSERVO3_PIN = 0;
    }
    if (Input & 0x08)
    {
        RCSERVO4_PIN = 1;
    }
    else
    {
        RCSERVO4_PIN = 0;
    }
    if (Input & 0x10)
    {
        RCSERVO5_PIN = 1;
    }
    else
    {
        RCSERVO5_PIN = 0;
    }
    if (Input & 0x20)
    {
        RCSERVO6_PIN = 1;
    }
    else
    {
        RCSERVO6_PIN = 0;
    }
    if (Input & 0x40)
    {
        RCSERVO7_PIN = 1;
    }
    else
    {
        RCSERVO7_PIN = 0;
    }
    if (Input & 0x80)
    {
        RCSERVO8_PIN = 1;
    }
    else
    {
        RCSERVO8_PIN = 0;
    }
}

// Reading from GPIO pins
UINT8 RCServoGPIORead(void)
{
    UINT8 Result = 0;
    if (RCSERVO1_PIN == 1)
    {
        Result = Result | 0x01;
    }
    if (RCSERVO2_PIN == 1)
    {
        Result = Result | 0x02;
    }
    if (RCSERVO3_PIN == 1)
    {
        Result = Result | 0x04;
    }
    if (RCSERVO4_PIN == 1)
    {
        Result = Result | 0x08;
    }
    if (RCSERVO5_PIN == 1)
    {
        Result = Result | 0x10;
    }
    if (RCSERVO6_PIN == 1)
    {
        Result = Result | 0x20;
    }
    if (RCSERVO7_PIN == 1)
    {
        Result = Result | 0x40;
    }
    if (RCSERVO8_PIN == 1)
    {
        Result = Result | 0x80;
    }

    return Result;
}

// Modify Input by GPIO invert bits and return value
UINT8 RCServoGPIOInvert(UINT8 Input)
{
    return (Input ^ RCServo_Invert);
}

void AnalogGPIOWrite(UINT8 Low, UINT8 High)
{
    if (Low & 0x01)
    {
        ANALOG1_PIN = 1;
    }
    else
    {
        ANALOG1_PIN = 0;
    }
    if (Low & 0x02)
    {
        ANALOG2_PIN = 1;
    }
    else
    {
        ANALOG2_PIN = 0;
    }
    if (Low & 0x04)
    {
        ANALOG3_PIN = 1;
    }
    else
    {
        ANALOG3_PIN = 0;
    }
    if (Low & 0x08)
    {
        ANALOG4_PIN = 1;
    }
    else
    {
        ANALOG4_PIN = 0;
    }
    if (Low & 0x10)
    {
        ANALOG5_PIN = 1;
    }
    else
    {
        ANALOG5_PIN = 0;
    }
    if (Low & 0x20)
    {
        ANALOG6_PIN = 1;
    }
    else
    {
        ANALOG6_PIN = 0;
    }
    if (Low & 0x40)
    {
        ANALOG7_PIN = 1;
    }
    else
    {
        ANALOG7_PIN = 0;
    }
    if (High & 0x01)
    {
        ANALOG8_PIN = 1;
    }
    else
    {
        ANALOG8_PIN = 0;
    }
    if (High & 0x02)
    {
        ANALOG9_PIN = 1;
    }
    else
    {
        ANALOG9_PIN = 0;
    }
    if (High & 0x04)
    {
        ANALOG10_PIN = 1;
    }
    else
    {
        ANALOG10_PIN = 0;
    }
    if (High & 0x08)
    {
        ANALOG11_PIN = 1;
    }
    else
    {
        ANALOG11_PIN = 0;
    }
    if (High & 0x10)
    {
        ANALOG12_PIN = 1;
    }
    else
    {
        ANALOG12_PIN = 0;
    }
    if (High & 0x20)
    {
        ANALOG13_PIN = 1;
    }
    else
    {
        ANALOG13_PIN = 0;
    }
    if (High & 0x40)
    {
        ANALOG14_PIN = 1;
    }
    else
    {
        ANALOG14_PIN = 0;
    }
}

// Read the analog I/O bits as GPIO inputs
// Take into account the analog invert bit values
void AnalogGPIORead(UINT8 * LowByte, UINT8 * HighByte)
{
    UINT8 Low = 0;
    UINT8 High = 0;
    if (ANALOG1_PIN == 1)
    {
        Low = Low | 0x01;
    }
    if (ANALOG2_PIN == 1)
    {
        Low = Low | 0x02;
    }
    if (ANALOG3_PIN == 1)
    {
        Low = Low | 0x04;
    }
    if (ANALOG4_PIN == 1)
    {
        Low = Low | 0x08;
    }
    if (ANALOG5_PIN == 1)
    {
        Low = Low | 0x10;
    }
    if (ANALOG6_PIN == 1)
    {
        Low = Low | 0x20;
    }
    if (ANALOG7_PIN == 1)
    {
        Low = Low | 0x40;
    }
    if (ANALOG8_PIN == 1)
    {
        High = High | 0x01;
    }
    if (ANALOG9_PIN == 1)
    {
        High = High | 0x02;
    }
    if (ANALOG10_PIN == 1)
    {
        High = High | 0x04;
    }
    if (ANALOG11_PIN == 1)
    {
        High = High | 0x08;
    }
    if (ANALOG12_PIN == 1)
    {
        High = High | 0x10;
    }
    if (ANALOG13_PIN == 1)
    {
        High = High | 0x20;
    }
    if (ANALOG14_PIN == 1)
    {
        High = High | 0x40;
    }

    // Now take invert into account
    *HighByte = High ^ Analog_Invert[1];
    *LowByte = Low ^ Analog_Invert[0];
}

// Take two bytes (high byte and low byte) and write them
// into the analog 'ports' direction bits.
void AnalogGPIOWriteDirection(UINT8 Low, UINT8 High)
{
    if (Low & 0x01)
    {
        ANALOG1_TRIS = 1;
    }
    else
    {
        ANALOG1_TRIS = 0;
    }
    if (Low & 0x02)
    {
        ANALOG2_TRIS = 1;
    }
    else
    {
        ANALOG2_TRIS = 0;
    }
    if (Low & 0x04)
    {
        ANALOG3_TRIS = 1;
    }
    else
    {
        ANALOG3_TRIS = 0;
    }
    if (Low & 0x08)
    {
        ANALOG4_TRIS = 1;
    }
    else
    {
        ANALOG4_TRIS = 0;
    }
    if (Low & 0x10)
    {
        ANALOG5_TRIS = 1;
    }
    else
    {
        ANALOG5_TRIS = 0;
    }
    if (Low & 0x20)
    {
        ANALOG6_TRIS = 1;
    }
    else
    {
        ANALOG6_TRIS = 0;
    }
    if (Low & 0x40)
    {
        ANALOG7_TRIS = 1;
    }
    else
    {
        ANALOG7_TRIS = 0;
    }
    if (High & 0x01)
    {
        ANALOG8_TRIS = 1;
    }
    else
    {
        ANALOG8_TRIS = 0;
    }
    if (High & 0x02)
    {
        ANALOG9_TRIS = 1;
    }
    else
    {
        ANALOG9_TRIS = 0;
    }
    if (High & 0x04)
    {
        ANALOG10_TRIS = 1;
    }
    else
    {
        ANALOG10_TRIS = 0;
    }
    if (High & 0x08)
    {
        ANALOG11_TRIS = 1;
    }
    else
    {
        ANALOG11_TRIS = 0;
    }
    if (High & 0x10)
    {
        ANALOG12_TRIS = 1;
    }
    else
    {
        ANALOG12_TRIS = 0;
    }
    if (High & 0x20)
    {
        ANALOG13_TRIS = 1;
    }
    else
    {
        ANALOG13_TRIS = 0;
    }
    if (High & 0x40)
    {
        ANALOG14_TRIS = 1;
    }
    else
    {
        ANALOG14_TRIS = 0;
    }

    Analog_Dir[0] = Low;
    Analog_Dir[1] = High;
}

// Read the TRIS bits from the analog bits and
// store them in the high and low bytes passed in
void AnalogGPIOReadDirection(UINT8 * LowByte, UINT8 * HighByte)
{
    UINT8 Low = 0;
    UINT8 High = 0;
    if (ANALOG1_TRIS == 1)
    {
        Low = Low | 0x01;
    }
    if (ANALOG2_TRIS == 1)
    {
        Low = Low | 0x02;
    }
    if (ANALOG3_TRIS == 1)
    {
        Low = Low | 0x04;
    }
    if (ANALOG4_TRIS == 1)
    {
        Low = Low | 0x08;
    }
    if (ANALOG5_TRIS == 1)
    {
        Low = Low | 0x10;
    }
    if (ANALOG6_TRIS == 1)
    {
        Low = Low | 0x20;
    }
    if (ANALOG7_TRIS == 1)
    {
        Low = Low | 0x40;
    }
    if (ANALOG8_TRIS == 1)
    {
        High = High | 0x01;
    }
    if (ANALOG9_TRIS == 1)
    {
        High = High | 0x02;
    }
    if (ANALOG10_TRIS == 1)
    {
        High = High | 0x04;
    }
    if (ANALOG11_TRIS == 1)
    {
        High = High | 0x08;
    }
    if (ANALOG12_TRIS == 1)
    {
        High = High | 0x10;
    }
    if (ANALOG13_TRIS == 1)
    {
        High = High | 0x20;
    }
    if (ANALOG14_TRIS == 1)
    {
        High = High | 0x40;
    }
    *LowByte = Low;
    *HighByte = High;
}

// Take a low and high enable byte values, and 
// set the  Analog_Enable[] values as well as
// ANCO0 and ANCON1 registers.
// Also set the specified bits to be inputs
void  AnalogWriteEnable(UINT8 Low, UINT8 High)
{
    Analog_Enable[0] = Low;
    Analog_Enable[1] = High;

    if (Low & 0x01)
    {
        ANALOG1_ENABLE = 1;
        ANALOG1_TRIS = INPUT;
    }
    else
    {
        ANALOG1_ENABLE = 0;
    }
    if (Low & 0x02)
    {
        ANALOG2_ENABLE = 1;
        ANALOG2_TRIS = INPUT;
    }
    else
    {
        ANALOG2_ENABLE = 0;
    }
    if (Low & 0x04)
    {
        ANALOG3_ENABLE = 1;
        ANALOG3_TRIS = INPUT;
    }
    else
    {
        ANALOG3_ENABLE = 0;
    }
    if (Low & 0x08)
    {
        ANALOG4_ENABLE = 1;
        ANALOG4_TRIS = INPUT;
    }
    else
    {
        ANALOG4_ENABLE = 0;
    }
    if (Low & 0x10)
    {
        ANALOG5_ENABLE = 1;
        ANALOG5_TRIS = INPUT;
    }
    else
    {
        ANALOG5_ENABLE = 0;
    }
    if (Low & 0x20)
    {
        ANALOG6_ENABLE = 1;
        ANALOG6_TRIS = INPUT;
    }
    else
    {
        ANALOG6_ENABLE = 0;
    }
    if (Low & 0x40)
    {
        ANALOG7_ENABLE = 1;
        ANALOG7_TRIS = INPUT;
    }
    else
    {
        ANALOG7_ENABLE = 0;
    }
    if (High & 0x01)
    {
        ANALOG8_ENABLE = 1;
        ANALOG8_TRIS = INPUT;
    }
    else
    {
        ANALOG8_ENABLE = 0;
    }
    if (High & 0x02)
    {
        ANALOG9_ENABLE = 1;
        ANALOG9_TRIS = INPUT;
    }
    else
    {
        ANALOG9_ENABLE = 0;
    }
    if (High & 0x04)
    {
        ANALOG10_ENABLE = 1;
        ANALOG10_TRIS = INPUT;
    }
    else
    {
        ANALOG10_ENABLE = 0;
    }
    if (High & 0x08)
    {
        ANALOG11_ENABLE = 1;
        ANALOG11_TRIS = INPUT;
    }
    else
    {
        ANALOG11_ENABLE = 0;
    }
    if (High & 0x10)
    {
        ANALOG12_ENABLE = 1;
        ANALOG12_TRIS = INPUT;
    }
    else
    {
        ANALOG12_ENABLE = 0;
    }
    if (High & 0x20)
    {
        ANALOG13_ENABLE = 1;
        ANALOG13_TRIS = INPUT;
    }
    else
    {
        ANALOG13_ENABLE = 0;
    }
    if (High & 0x40)
    {
        ANALOG14_ENABLE = 1;
        ANALOG14_TRIS = INPUT;
    }
    else
    {
        ANALOG14_ENABLE = 0;
    }
}

// Take in an analog pin number - 1 (0 through 13)
// return the PIC analog channel assigned to that pin
UINT8 AnalogChannelMap(UINT8 Input)
{
    return(AnalogPinMap[Input]);
}

