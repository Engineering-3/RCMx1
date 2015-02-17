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

#include <p18f86k22.h>
#include <GenericTypeDefs.h>

#ifndef __SYSTEM_H__
#define __SYSTEM_H__

/******************************************************************************/
/* System Level #define Macros                                                */
/******************************************************************************/

/* TODO Define system operating frequency */

/* Microcontroller MIPs (FCY) */
#define SYS_FREQ        8000000L
#define FCY             SYS_FREQ/4

#define OUTPUT          0
#define INPUT           1

// What I2C address do we want to respond to?
#define RCMX1_I2C_ADDRESS   0x38

// Our firmware version number (1 byte)
#define RCMX1_FIMRWARE_VERSION  0x07

#define RCMx1_PORTB_MASK        0b00111111
#define RCMx1_PORTB_RBPU_MASK   0b00000001

#define RC_SERVO_COUNT       8

#define GPIO_PULLUP_ENABLE      0
#define GPIO_PULLUP_DISABLE     1

// For debug output bits
#define DEBUG1    LATCbits.LATC6
#define DEBUG2    LATCbits.LATC7
#define DEBUG3    LATDbits.LATD0
#define DEBUG4    LATDbits.LATD1
#define DEBUG5    LATDbits.LATD2
#define DEBUG6    LATDbits.LATD3
#define DEBUG7    LATDbits.LATD4
#define DEBUG8    LATDbits.LATD7

#define DEBUG1_TRIS    TRISCbits.TRISC6
#define DEBUG2_TRIS    TRISCbits.TRISC7
#define DEBUG3_TRIS    TRISDbits.TRISD0
#define DEBUG4_TRIS    TRISDbits.TRISD1
#define DEBUG5_TRIS    TRISDbits.TRISD2
#define DEBUG6_TRIS    TRISDbits.TRISD3
#define DEBUG7_TRIS    TRISDbits.TRISD4
#define DEBUG8_TRIS    TRISDbits.TRISD7

// Define our pins
#define RCSERVO1_PIN    PORTCbits.RC2      // RC_SERVO_1
#define RCSERVO2_PIN    PORTCbits.RC3      // RC_SERVO_2
#define RCSERVO3_PIN    PORTCbits.RC4      // RC_SERVO_3
#define RCSERVO4_PIN    PORTCbits.RC5      // RC_SERVO_4
#define RCSERVO5_PIN    PORTAbits.RA6      // RC_SERVO_5
#define RCSERVO6_PIN    PORTAbits.RA7      // RC_SERVO_6
#define RCSERVO7_PIN    PORTCbits.RC0      // RC_SERVO_7
#define RCSERVO8_PIN    PORTCbits.RC1      // RC_SERVO_8

#define RCSERVO1_TRIS   TRISCbits.TRISC2    // RC_SERVO_1
#define RCSERVO2_TRIS   TRISCbits.TRISC3    // RC_SERVO_2
#define RCSERVO3_TRIS   TRISCbits.TRISC4    // RC_SERVO_3
#define RCSERVO4_TRIS   TRISCbits.TRISC5    // RC_SERVO_4
#define RCSERVO5_TRIS   TRISAbits.TRISA6    // RC_SERVO_5
#define RCSERVO6_TRIS   TRISAbits.TRISA7    // RC_SERVO_6
#define RCSERVO7_TRIS   TRISCbits.TRISC0    // RC_SERVO_7
#define RCSERVO8_TRIS   TRISCbits.TRISC1    // RC_SERVO_8

#define ANALOG1_PIN     PORTGbits.RG1    // LCA_L1
#define ANALOG2_PIN     PORTFbits.RF7    // LCA_L2
#define ANALOG3_PIN     PORTFbits.RF5    // LCA_L3
#define ANALOG4_PIN     PORTFbits.RF3    // LCA_L4
#define ANALOG5_PIN     PORTFbits.RF1    // LCA_L5
#define ANALOG6_PIN     PORTAbits.RA1    // HCA_L1
#define ANALOG7_PIN     PORTAbits.RA3    // HCA_L2
#define ANALOG8_PIN     PORTGbits.RG2    // LCA_R1
#define ANALOG9_PIN     PORTFbits.RF6    // LCA_R2
#define ANALOG10_PIN    PORTFbits.RF4    // LCA_R3
#define ANALOG11_PIN    PORTFbits.RF2    // LCA_R4
#define ANALOG12_PIN    PORTAbits.RA0    // HCA_R1
#define ANALOG13_PIN    PORTAbits.RA2    // HCA_R2
#define ANALOG14_PIN    PORTAbits.RA5    // HCA_R3

#define ANALOG1_TRIS    TRISGbits.TRISG1    // LCA_L1
#define ANALOG2_TRIS    TRISFbits.TRISF7    // LCA_L2
#define ANALOG3_TRIS    TRISFbits.TRISF5    // LCA_L3
#define ANALOG4_TRIS    TRISFbits.TRISF3    // LCA_L4
#define ANALOG5_TRIS    TRISFbits.TRISF1    // LCA_L5
#define ANALOG6_TRIS    TRISAbits.TRISA1    // HCA_L1
#define ANALOG7_TRIS    TRISAbits.TRISA3    // HCA_L2
#define ANALOG8_TRIS    TRISGbits.TRISG2    // LCA_R1
#define ANALOG9_TRIS    TRISFbits.TRISF6    // LCA_R2
#define ANALOG10_TRIS   TRISFbits.TRISF4    // LCA_R3
#define ANALOG11_TRIS   TRISFbits.TRISF2    // LCA_R4
#define ANALOG12_TRIS   TRISAbits.TRISA0    // HCA_R1
#define ANALOG13_TRIS   TRISAbits.TRISA2    // HCA_R2
#define ANALOG14_TRIS   TRISAbits.TRISA5    // HCA_R3

#define ANALOG1_ENABLE  ANCON2bits.ANSEL19  // LCA_L1
#define ANALOG2_ENABLE  ANCON0bits.ANSEL5   // LCA_L2
#define ANALOG3_ENABLE  ANCON1bits.ANSEL10  // LCA_L3
#define ANALOG4_ENABLE  ANCON1bits.ANSEL8   // LCA_L4
#define ANALOG5_ENABLE  ANCON0bits.ANSEL6   // LCA_L5
#define ANALOG6_ENABLE  ANCON0bits.ANSEL1   // HCA_L1
#define ANALOG7_ENABLE  ANCON0bits.ANSEL3   // HCA_L2
#define ANALOG8_ENABLE  ANCON2bits.ANSEL18  // LCA_R1
#define ANALOG9_ENABLE  ANCON1bits.ANSEL11  // LCA_R2
#define ANALOG10_ENABLE ANCON1bits.ANSEL9   // LCA_R3
#define ANALOG11_ENABLE ANCON0bits.ANSEL7   // LCA_R4
#define ANALOG12_ENABLE ANCON0bits.ANSEL0   // HCA_R1
#define ANALOG13_ENABLE ANCON0bits.ANSEL2   // HCA_R2
#define ANALOG14_ENABLE ANCON0bits.ANSEL4   // HCA_R3

// Maps GPIO numbers 1 through 6 to actual positions within PORTB pins
#define GPIO1_BIT       1<<1
#define GPIO2_BIT       1<<3
#define GPIO3_BIT       1<<5
#define GPIO4_BIT       1
#define GPIO5_BIT       1<<2
#define GPIO6_BIT       1<<4



/******************************************************************************/
/* System Function Prototypes                                                 */
/******************************************************************************/

/* Custom oscillator configuration funtions, reset source evaluation
functions, and other non-peripheral microcontroller initialization functions
go here. */

void  ConfigureOscillator(void); /* Handles clock switching/osc initialization */
void  IOInit(void);
UINT8 GPIOPinMapOut(UINT8 Input);
UINT8 GPIOPinMapIn(UINT8 Input);
UINT8 GPIOInvert(UINT8 Input);
UINT8 RCServoGPIORead(void);
void  RCServoGPIOWrite(UINT8 Input);
UINT8 RCServoGPIOInvert(UINT8 Input);
UINT8 AnalogChannelMap(UINT8 Input);
UINT8 RCServoGPIOReadDirection(void);
void  RCServoGPIOWriteDirection(UINT8 Input);
void  AnalogGPIOWrite(UINT8 Low, UINT8 High);
void  AnalogGPIORead(UINT8 * Low, UINT8 * High);
void  AnalogGPIOReadDirection(UINT8 * Low, UINT8 * High);
void  AnalogGPIOWriteDirection(UINT8 Low, UINT8 High);
void  AnalogWriteEnable(UINT8 Low, UINT8 High);

#endif