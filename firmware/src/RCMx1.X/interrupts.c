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
#include "I2C.h"
#include "interrupts.h"
#include "user.h"
#include "PWM.h"
#include "timers.h"

/******************************************************************************/
/* Interrupt Routines                                                         */
/******************************************************************************/
void InterruptHandlerHigh(void);
void InterruptHandlerLow(void);

void InterruptsInit(void)
{
    // Enable high and low prioritiy interrupts
    RCONbits.IPEN = 1;

    // Turn on low interrupt
    INTCONbits.GIEL = 1;

    // Turn on high interrupt
    INTCONbits.GIEH = 1;
}

/* High-priority service */
#pragma code InterrutpVectorHigh = 0x08

void InterruptVectorHigh(void)
{
    _asm
    goto InterruptHandlerHigh
        _endasm
}

/* Low-priority service */
#pragma code InterrutpVectorLow = 0x18

void InterruptVectorLow(void)
{
    _asm
    goto InterruptHandlerLow
        _endasm
}

#pragma code
#pragma interrupt InterruptHandlerHigh

void InterruptHandlerHigh(void)
{
    INT16 ErrorSig = 0;
    UINT8 i;
    UINT16 NextTime = 0;
    UINT16 CurrentTime = 0;
    // Store the current place we are in the 24ms frame
    static UINT16 CurrentFrameTime = 0;

    // Was it the RC Servo Timer3?
    if (PIR2bits.TMR3IF)
    {
        // If our current channel's I/O pin is on, then turn it off
        if (RCServo_SignalOn)
        {
            // Turn off the I/O pin
            switch (RCServo_CurrentChannel)
            {
            case 0:
                RCSERVO1_PIN = 0;
                break;
            case 1:
                RCSERVO2_PIN = 0;
                break;
            case 2:
                RCSERVO3_PIN = 0;
                break;
            case 3:
                RCSERVO4_PIN = 0;
                break;
            case 4:
                RCSERVO5_PIN = 0;
                break;
            case 5:
                RCSERVO6_PIN = 0;
                break;
            case 6:
                RCSERVO7_PIN = 0;
                break;
            case 7:
                RCSERVO8_PIN = 0;
                break;
            default:
                break;
            }
            RCServo_SignalOn = FALSE;

            // We need to fire the ISR at the next pin time (normally 3 ms from)
            // the leading edge of this pin.
            /// TODO: Could this be pre-computed?
            NextTime = 0xFFFF - (RC_SERVO_NEXT_PIN_TIME - RCServo_Width[RCServo_CurrentChannel]);
            // Grab the current time, which will be the number of ticks we've gone past zero
            TMR3H = NextTime >> 8;
            TMR3L = NextTime & 0xFF;
            CurrentFrameTime += (RC_SERVO_NEXT_PIN_TIME - RCServo_Width[RCServo_CurrentChannel]);

            // Since we're done with this channel, advance to the next one
            RCServo_CurrentChannel++;
        }
        else
        {
            // We need to find the next PWM channel that's enabled
            while (RCServo_CurrentChannel < RC_SERVO_COUNT)
            {
                if (RCServo_Enable[RCServo_CurrentChannel] == RC_SERVO_ENABLE_ON)
                {
                    // If slow move is enabled, take care of moveing the target
                    if (RCServo_SlowMove[RCServo_CurrentChannel] != 0x00)
                    {
                        // Only do stuff if we're not at the edges of travel
                        if (
                            (RCServo_TargetWidth[RCServo_CurrentChannel] < RCServo_MaxWidth[RCServo_CurrentChannel])
                            &&
                            (RCServo_SlowMove[RCServo_CurrentChannel] > 0x80)
                            )
                        {
                            RCServo_TargetWidth[RCServo_CurrentChannel] += (RCServo_SlowMove[RCServo_CurrentChannel] - 0x80) * SLOW_MOVE_MULT_FACTOR;
                        }
                        if (
                            (RCServo_TargetWidth[RCServo_CurrentChannel] > RCServo_MinWidth[RCServo_CurrentChannel])
                            &&
                            (RCServo_SlowMove[RCServo_CurrentChannel] < 0x80)
                            )
                        {
                            RCServo_TargetWidth[RCServo_CurrentChannel] -= (0x80 - RCServo_SlowMove[RCServo_CurrentChannel]) * SLOW_MOVE_MULT_FACTOR;
                        }

                        // Limit to Min/Max
                        if (RCServo_TargetWidth[RCServo_CurrentChannel] < RCServo_MinWidth[RCServo_CurrentChannel])
                        {
                            RCServo_TargetWidth[RCServo_CurrentChannel] = RCServo_MinWidth[RCServo_CurrentChannel];
                        }
                        if (RCServo_TargetWidth[RCServo_CurrentChannel] > RCServo_MaxWidth[RCServo_CurrentChannel])
                        {
                            RCServo_TargetWidth[RCServo_CurrentChannel] = RCServo_MaxWidth[RCServo_CurrentChannel];
                        }
                    }

                    // Update RCServo_Width based on RCServo_TargetWidth
                    if (RCServo_FilterEnabled[RCServo_CurrentChannel] == RC_SERVO_FILTER_ON)
                    {
                        // Compute error signal - difference from target and actual
                        // Will be positive if we are trying to go more forward, neg if we're trying to reverse
                        ErrorSig = RCServo_TargetWidth[RCServo_CurrentChannel] - RCServo_Width[RCServo_CurrentChannel];

                        // Limit the difference between this new pulse width and the target to
                        // max values (separate for forward and reverse directions)
                        if (ErrorSig > RCServo_MaxAccel[RCServo_CurrentChannel])
                        {
                            ErrorSig = RCServo_MaxAccel[RCServo_CurrentChannel];
                        }
                        if (ErrorSig < -((INT16)RCServo_MaxDecel[RCServo_CurrentChannel]))
                        {
                            ErrorSig = -((INT16)RCServo_MaxDecel[RCServo_CurrentChannel]);
                        }

                        // Move a little bit towards our target width
                        RCServo_Width[RCServo_CurrentChannel] += ErrorSig;

                        // Limit check the actual servo positions
                        if (RCServo_Width[RCServo_CurrentChannel] > RCServo_MaxForward[RCServo_CurrentChannel])
                        {
                            RCServo_Width[RCServo_CurrentChannel] = RCServo_MaxForward[RCServo_CurrentChannel];
                        }
                        if (RCServo_Width[RCServo_CurrentChannel] < RCServo_MaxReverse[RCServo_CurrentChannel])
                        {
                            RCServo_Width[RCServo_CurrentChannel] = RCServo_MaxReverse[RCServo_CurrentChannel];
                        }
                    }
                    else
                    {
                        // When the filter is not enabled, just copy over the target to the current width
                        RCServo_Width[RCServo_CurrentChannel] = RCServo_TargetWidth[RCServo_CurrentChannel];
                    }

                    // Turn on the I/O pin
                    switch (RCServo_CurrentChannel)
                    {
                    case 0:
                        RCSERVO1_PIN = 1;
                        break;
                    case 1:
                        RCSERVO2_PIN = 1;
                        break;
                    case 2:
                        RCSERVO3_PIN = 1;
                        break;
                    case 3:
                        RCSERVO4_PIN = 1;
                        break;
                    case 4:
                        RCSERVO5_PIN = 1;
                        break;
                    case 5:
                        RCSERVO6_PIN = 1;
                        break;
                    case 6:
                        RCSERVO7_PIN = 1;
                        break;
                    case 7:
                        RCSERVO8_PIN = 1;
                        break;
                    default:
                        break;
                    }

                    // And set us to re-fire at the next I/O pin's time
                    NextTime = 0xFFFF - RCServo_Width[RCServo_CurrentChannel];
                    TMR3H = NextTime >> 8;
                    TMR3L = NextTime & 0xFF;
                    CurrentFrameTime += RCServo_Width[RCServo_CurrentChannel];

                    RCServo_SignalOn = TRUE;
                    break;
                }
                RCServo_CurrentChannel++;
            }
            // If there are no more enabled channels,
            // then set the next fire to happen at the beginning of the next frame
            if (RCServo_SignalOn == FALSE)
            {
                RCServo_CurrentChannel = 0;
                NextTime = 0xFFFF - (RC_SERVO_FRAME_TIME - CurrentFrameTime);
                TMR3H = NextTime >> 8;
                TMR3L = NextTime & 0xFF;
                CurrentFrameTime = 0;
            }
        }
        PIR2bits.TMR3IF = 0;
    }
        /* Unhandled interrupts */
    else
    {
        while (1)
        {
            ;
        }
    }
}

#pragma interrupt InterruptHandlerLow

void InterruptHandlerLow(void)
{
    static UINT8 Address = 0;
    static UINT8 DataPointer = 0;
    static UINT8 Data[10];
    static UINT8 Register = 0;
    static UINT8 SecondByte = 0;
    static UINT8 Analog_Channel = 0;
    static UINT8 Analog_Index = 0;
    static UINT16 OneSecondCounter = 0;

    /* Determine which flag generated the interrupt */
    /* Was it I2C? */
    if (PIR2bits.SSP2IF)
    {
        // If we have a byte of data
        if (SSP2STATbits.BF == 1)
        {
            // And it's an address
            if (SSP2STATbits.D_A == 0)
            {
                // Then we just started a transaction, and we just saw our address on
                // the bus. Record the address (not really needed since we only use
                // one I2C address)
                Address = SSP2BUF;
                // Check for SSPOV and clear if set
                if (SSP2CON1bits.SSPOV)
                {
                    // Error here - we NAKed the master's address for some reason
                    // Clear the error bit
                    SSP2CON1bits.SSPOV = 0;
                }

                // Reset the data pointer since this is the beginning
                DataPointer = 0;

                // If the master is writing to us, clear the 'register' variable
                if (SSP2STATbits.R_W == 0)
                {
                    Register = 0x00;
                }
                else
                {
                    // Reset the safety command timeout
                    OneSecondCounter = 0;
                    LastCommandTime = 0;

                    SecondByte = 0;
                    // If this is the master reading from us, send it the first byte of the
                    // data.
                    switch (Register)
                    {
                        // Version number, 1 byte
                    case 0x00:
                        SSP2BUF = RCMX1_FIMRWARE_VERSION;
                        break;

                        // Safety Timeout Value
                    case 0x0E:
                        SSP2BUF = (UINT8)(SafetyTimeoutValue/1000);
                        break;

                        // Safety Timeout Reset
                    case 0x0F:
                        SSP2BUF = RCMX1_FIMRWARE_VERSION;
                        break;

                        // GPIO read value, 1 byte
                    case 0x10:
                        // Execute the read of the I/O ports here
                        // Store into the array
                        GPIO_IO = GPIOInvert(GPIOPinMapIn(PORTB & RCMx1_PORTB_MASK));
                        SSP2BUF = GPIO_IO;
                        break;

                        // GPIO read direction, 1 byte
                    case 0x11:
                        // Execute the read of the GPIO direction bits
                        // As of version 0.8, we now used inverted logic for this register
                        GPIO_Dir = GPIOPinMapIn(~(TRISB & RCMx1_PORTB_MASK));
                        SSP2BUF = GPIO_Dir;
                        break;

                        // GPIO read pull up, 1 byte
                    case 0x12:
                        // Read the pull up bit
                        SSP2BUF = GPIO_PullUp;
                        break;

                        // GPIO read invert control, 1 byte
                    case 0x13:
                        // Execute the read of the GPIO invert bits
                        SSP2BUF = GPIO_Invert & RCMx1_PORTB_MASK;
                        break;

                        // RC Servo read value, 1 byte
                    case 0x20:
                        // Execute the read of the RC Servo I/O pins here
                        RCServo_IO = RCServoGPIOInvert(RCServoGPIORead());
                        SSP2BUF = RCServo_IO;
                        break;

                        // RC Servo read direction, 1 byte
                    case 0x21:
                        SSP2BUF = RCServo_Dir;
                        break;

                        // RCServo read invert control, 1 byte
                    case 0x23:
                        // Send robot our current invert byte
                        SSP2BUF = RCServo_Invert;
                        break;

                        // RCServo read enable bits, 1 byte
                    case 0x24:
                        // Send the RC Servo enable bits
                        SSP2BUF =
                            RCServo_Enable[0]
                            |
                            RCServo_Enable[1] << 1
                            |
                            RCServo_Enable[2] << 2
                            |
                            RCServo_Enable[3] << 3
                            |
                            RCServo_Enable[4] << 4
                            |
                            RCServo_Enable[5] << 5
                            |
                            RCServo_Enable[6] << 6
                            |
                            RCServo_Enable[7] << 7;
                        break;

                        // RCServo read filter enable control, 1 byte
                    case 0x25:
                        // Execute the read of the RCServo filter enable bits
                        SSP2BUF =
                            ((RCServo_FilterEnabled[0] == RC_SERVO_FILTER_ON)?1:0)
                            |
                            (((RCServo_FilterEnabled[1] << 1) == RC_SERVO_FILTER_ON)?1:0)
                            |
                            (((RCServo_FilterEnabled[2] << 2) == RC_SERVO_FILTER_ON)?1:0)
                            |
                            (((RCServo_FilterEnabled[3] << 3) == RC_SERVO_FILTER_ON)?1:0)
                            |
                            (((RCServo_FilterEnabled[4] << 4) == RC_SERVO_FILTER_ON)?1:0)
                            |
                            (((RCServo_FilterEnabled[5] << 5) == RC_SERVO_FILTER_ON)?1:0)
                            |
                            (((RCServo_FilterEnabled[6] << 6) == RC_SERVO_FILTER_ON)?1:0)
                            |
                            (((RCServo_FilterEnabled[7] << 7) == RC_SERVO_FILTER_ON)?1:0);
                        break;

                        // RCServo read position bytes, 1 byte
                    case 0x31:
                    case 0x32:
                    case 0x33:
                    case 0x34:
                    case 0x35:
                    case 0x36:
                    case 0x37:
                    case 0x38:
                        // Send the RC Servo position bytes
                        SSP2BUF = RCServo_Unscale(RCServo_TargetWidth[Register - 0x31], Register - 0x31);
                        break;

                        // Send the RC Servo max forward speed bytes
                    case 0x41:
                    case 0x42:
                    case 0x43:
                    case 0x44:
                    case 0x45:
                    case 0x46:
                    case 0x47:
                    case 0x48:
                        // Send the RC Servo max forward speed bytes
                        SSP2BUF = RCServo_Unscale(RCServo_MaxForward[Register - 0x41], Register - 0x41);
                        break;

                        // Send the RC Servo max reverse bytes
                    case 0x51:
                    case 0x52:
                    case 0x53:
                    case 0x54:
                    case 0x55:
                    case 0x56:
                    case 0x57:
                    case 0x58:
                        // Send the RC Servo max reverse bytes
                        SSP2BUF = RCServo_Unscale(RCServo_MaxReverse[Register - 0x51], Register - 0x51);
                        break;

                        // Send the RC Servo max acceleration bytes
                    case 0x61:
                    case 0x62:
                    case 0x63:
                    case 0x64:
                    case 0x65:
                    case 0x66:
                    case 0x67:
                    case 0x68:
                        // Send the RC Servo max acceleration bytes
                        SSP2BUF = RCServo_AccelUnScale(RCServo_MaxAccel[Register - 0x61]);
                        break;

                        // Send the RC Servo max deceleration bytes
                    case 0x71:
                    case 0x72:
                    case 0x73:
                    case 0x74:
                    case 0x75:
                    case 0x76:
                    case 0x77:
                    case 0x78:
                        // Send the RC Servo max deceleration bytes
                        SSP2BUF = RCServo_AccelUnScale(RCServo_MaxDecel[Register - 0x71]);
                        break;

                        // Send the RC Servo slow move bytes
                    case 0x91:
                    case 0x92:
                    case 0x93:
                    case 0x94:
                    case 0x95:
                    case 0x96:
                    case 0x97:
                    case 0x98:
                        // Send the RC Servo slow move bytes
                        SSP2BUF = RCServo_SlowMove[Register - 0x91];
                        break;

                        // Analog pins read value, 2 bytes
                    case 0xA0:
                        // Execute the read of the Analog ports here
                        // Store into the array
                        AnalogGPIORead(&Analog_IO[0], &Analog_IO[1]);

                        SSP2BUF = Analog_IO[1];
                        SecondByte = Analog_IO[0];
                        break;

                        // Analog read direction, 2 bytes
                    case 0xA1:
                        // Execute the read of the Analog direction bits
                        AnalogGPIOReadDirection(&Analog_Dir[0], &Analog_Dir[1]);
                        SSP2BUF = Analog_Dir[1];
                        SecondByte = Analog_Dir[0];
                        break;

                        // Analog read invert control, 2 bytes
                    case 0xA3:
                        // Execute the read of the Analog invert bits
                        SSP2BUF = Analog_Invert[1];
                        SecondByte = Analog_Invert[0];
                        break;

                        // Analog read analog/GPIO enable bits, 2 bytes
                    case 0xA4:
                        // Execute the read of the Analog enable bits
                        SSP2BUF = Analog_Enable[1];
                        SecondByte = Analog_Enable[0];
                        break;

                        // Analog read 12-bit values
                    case 0xB1:
                    case 0xB2:
                    case 0xB3:
                    case 0xB4:
                    case 0xB5:
                    case 0xB6:
                    case 0xB7:
                        // Send analog values (2-bytes), but only if the
                        // corresponding bit in 0xA4 is clear.
                        if (~Analog_Enable[0] & (0x01 << (Register - 0xB1)) ) {
                            SSP2BUF = Analog_Value[Register - 0xB1] >> 8;
                            SecondByte = Analog_Value[Register - 0xB1] & 0xFF;
                        }
                        else {
                            SSP2BUF = 0;
                            SecondByte = 0;
                        }
                        break;

                    case 0xB8:
                    case 0xB9:
                    case 0xBA:
                    case 0xBB:
                    case 0xBC:
                    case 0xBD:
                    case 0xBE:
                        // Send analog values (2-bytes), but only if the
                        // corresponding bit in 0xA4 is clear.
                        if (~Analog_Enable[1] & (0x01 << (Register - 0xB8)) ) {
                            SSP2BUF = Analog_Value[Register - 0xB1] >> 8;
                            SecondByte = Analog_Value[Register - 0xB1] & 0xFF;
                        }
                        else {
                            SSP2BUF = 0;
                            SecondByte = 0;
                        }
                        break;

                        // Send motor driver speed values
                    case 0xD1:
                    case 0xD2:
                    case 0xD3:
                    case 0xD4:
                        // Send motor driver speed values (1-byte)
                        SSP2BUF = Motor_Value[Register - 0xD1];
                        break;

                        // Values to send if an unknown register address is used
                    default:
                        SSP2BUF = 0xFF;
                        SecondByte = 0xFF;
                        break;
                    }
                    // Enable the clock line (disable stretching)
                    SSP2CON1bits.CKP = 1;
                }
            }
            else
                // Otherwise it's a byte of data
            {
                // Is master writing to us?
                if (SSP2STATbits.R_W == 0)
                {
                    // Reset the safety command timeout
                    OneSecondCounter = 0;
                    LastCommandTime = 0;

                    if (DataPointer == 0)
                    {
                        // If this is the first byte of data that the master sent us, then
                        // it is the 'register' it wants to read/write from so save it off
                        Register = SSP2BUF;
                    }
                    else
                    {
                        Data[DataPointer] = SSP2BUF;
                    }
                    if (DataPointer < 10)
                    {
                        DataPointer++;
                    }

                    // Check for SSPOV and clear if set
                    if (SSP2CON1bits.SSPOV == 1)
                    {
                        // Error here - we NAKed the master's address for some reason
                        // Clear the error bit
                        SSP2CON1bits.SSPOV = 0;
                    }

                    // If we've just received the first data byte, and we have a one byte
                    // command, then process it
                    if (DataPointer == 2)
                    {
                        switch (Register)
                        {
                            // Safety Timeout Value write, 1 byte
                        case 0x0E:
                            SafetyTimeoutValue = Data[1]*1000;
                            break;

                            // GPIO write data, 1 byte
                        case 0x10:
                            GPIO_IO = Data[1] & RCMx1_PORTB_MASK;
                            // Write out data to GPIO ports
                            LATB = (LATB & ~RCMx1_PORTB_MASK) | GPIOPinMapOut(GPIO_IO);
                            break;

                            // GPIO direction write, 1 byte
                        case 0x11:
                            GPIO_Dir = Data[1] & RCMx1_PORTB_MASK;
                            // Write out direction to GPIO ports
                            // As of version 0.8, we now used inverted logic for this register
                            TRISB = (TRISB & ~RCMx1_PORTB_MASK) | ~(GPIOPinMapOut(GPIO_Dir));
                            break;

                            // GPIO write pull up, 1 byte
                        case 0x12:
                            // Set pull up bits appropriately (on or off for whole port)
                            GPIO_PullUp = Data[1] & 0x01;
                            if (GPIO_PullUp)
                            {
                                INTCON2bits.RBPU = GPIO_PULLUP_ENABLE;
                            }
                            else
                            {
                                INTCON2bits.RBPU = GPIO_PULLUP_DISABLE;
                            }
                            break;

                            // GPIO write invert control, 1 byte
                        case 0x13:
                            GPIO_Invert = Data[1] & RCMx1_PORTB_MASK;
                            break;

                            // RC Servo GPIO data write, 1 byte
                        case 0x20:
                            RCServo_IO = Data[1];
                            // Write to RC servo port(s)
                            RCServoGPIOWrite(RCServo_IO);
                            break;

                            // RC Servo direction bits, 1 byte
                        case 0x21:
                            RCServo_Dir = Data[1];
                            // Set direction bits appropriately
                            RCServoGPIOWriteDirection(RCServo_Dir);
                            break;

                            // RC Servo invert control, 1 byte
                        case 0x23:
                            RCServo_Invert = Data[1];
                            break;

                            // RC Servo enable bits write, 1 byte
                        case 0x24:
                            RCServoSetEnables(Data[1]);
                            RCServoGPIOWriteDirection(RCServo_Dir);
                            break;

                            // RC Servo filter enable bits write, 1 byte
                        case 0x25:
                            RCServoSetFilterEnables(Data[1]);
                            break;

                            // RC Servo motor position value write, 1 byte
                        case 0x31:
                        case 0x32:
                        case 0x33:
                        case 0x34:
                        case 0x35:
                        case 0x36:
                        case 0x37:
                        case 0x38:
                            // If this was a zero data byte (0x00) then disable the servo
                            if (Data[1] == 0x00)
                            {
                                RCServo_Enable[Register - 0x31] = RC_SERVO_ENABLE_OFF;
                                RCServoGPIOWriteDirection(RCServo_Dir);
                            }
                            else
                            {
                                RCServo_Enable[Register - 0x31] = RC_SERVO_ENABLE_ON;
                                RCServo_TargetWidth[Register - 0x31] = RCServo_Scale(Data[1], Register - 0x31);
                                RCServoGPIOWriteDirection(RCServo_Dir);
                            }
                            break;

                            // RC Servo max forward speed value write, 1 byte
                        case 0x41:
                        case 0x42:
                        case 0x43:
                        case 0x44:
                        case 0x45:
                        case 0x46:
                        case 0x47:
                        case 0x48:
                            RCServo_MaxForward[Register - 0x41] = RCServo_Scale(Data[1], Register - 0x41);
                            break;

                            // RC Servo max revers speed value write, 1 byte
                        case 0x51:
                        case 0x52:
                        case 0x53:
                        case 0x54:
                        case 0x55:
                        case 0x56:
                        case 0x57:
                        case 0x58:
                            RCServo_MaxReverse[Register - 0x51] = RCServo_Scale(Data[1], Register - 0x51);
                            break;

                            // RC Servo max acceleration value write, 1 byte
                        case 0x61:
                        case 0x62:
                        case 0x63:
                        case 0x64:
                        case 0x65:
                        case 0x66:
                        case 0x67:
                        case 0x68:
                            // 0x00 is not an allowed value here.
                            if (Data[1] == 0x00) {
                                Data[1] = 0x1;
                            }
                            RCServo_MaxAccel[Register - 0x61] = RCServo_AccelScale(Data[1]);
                            break;

                            // RC Servo max deceleration value write, 1 byte
                        case 0x71:
                        case 0x72:
                        case 0x73:
                        case 0x74:
                        case 0x75:
                        case 0x76:
                        case 0x77:
                        case 0x78:
                            // 0x00 is not an allowed value here.
                            if (Data[1] == 0x00) {
                                Data[1] = 0x1;
                            }
                            RCServo_MaxDecel[Register - 0x71] = RCServo_AccelScale(Data[1]);
                            break;

                            // RC Servo slow move value write, 1 byte
                        case 0x91:
                        case 0x92:
                        case 0x93:
                        case 0x94:
                        case 0x95:
                        case 0x96:
                        case 0x97:
                        case 0x98:
                            RCServo_SlowMove[Register - 0x91] = Data[1];
                            break;

                            // Motor Driver position value, 1 byte
                        case 0xD1:
                        case 0xD2:
                        case 0xD3:
                        case 0xD4:
                            Motor_Value[Register - 0xD1] = Data[1];
                            // Now update the motor's PWM hardware
                            PWMUpdateValue(Register - 0xD1, Data[1]);
                            break;

                            // Do nothing for unknown registers
                        default:
                            break;
                        }
                    }
                    // If we've just received the 2nd data byte, then process
                    // any commands that need 2 data bytes.
                    if (DataPointer == 3)
                    {
                        switch (Register)
                        {
                            // Analog input as GPIO data write, 2 bytes
                        case 0xA0:
                            Analog_IO[0] = Data[1];
                            Analog_IO[1] = Data[2];
                            AnalogGPIOWrite(Analog_IO[0], Analog_IO[1]);
                            break;

                            // Analog input as GPIO direction write, 2 bytes
                        case 0xA1:
                            AnalogGPIOWriteDirection(Data[1], Data[2]);
                            break;

                            // Analog input as GPIO invert control write, 2 bytes
                        case 0xA3:
                            Analog_Invert[0] = Data[1];
                            Analog_Invert[1] = Data[2];
                            break;

                            // Analog input as GPIO enable control write, 2 bytes
                        case 0xA4:
                            // This takes care of writing to Analog_Enable[] as well
                            AnalogWriteEnable(Data[1], Data[2]);
                            break;

                            // Do nothing for unknown registers
                        default:
                            break;
                        }
                    }
                }
            }
        }
        if (SSP2STATbits.D_A == 1 && SSP2STATbits.R_W == 1 && SSP2STATbits.BF == 0)
        {
            // We only have commands that are up to 2 bytes long. So we
            // have already cached the second byte to send. So send it.
            SSP2BUF = SecondByte;
            Nop();
            Nop();
            Nop();
            Nop();
            Nop();
            Nop();
            Nop();
            Nop();
            Nop();
            Nop();
            Nop();
            Nop();
            Nop();
            Nop();
            // Enable the clock line (disable stretching)
            SSP2CON1bits.CKP = 1;
        }
        PIR2bits.SSP2IF = 0; /* Clear Interrupt Flag 1 */
    }

    // The Timer4 interrupt fires every 1ms
    else if (PIR5bits.TMR4IF)
    {
        UINT8 i;
        INT16 Temp;
        PIR5bits.TMR4IF = 0; // Clear the interrupt flag

        // Read out the current ADC value
        Temp = (((UINT16) ADRESH) << 8) | ADRESL;
        // If we get a negative value, just call it zero.
        if (Temp < 0)
        {
            Temp = 0;
        }
        // Store the new value in the proper register spot
        Analog_Value[Analog_Index] = Temp;
        // Move our index to the next analog channel
        Analog_Index++;
        if (Analog_Index >= 14)
        {
            Analog_Index = 0;
        }
        // Map an analog index number to an actual PIC analog channel
        Analog_Channel = AnalogChannelMap(Analog_Index);
        // Select the next channel
        ADCON0bits.CHS = Analog_Channel;
        // And start the next conversion
        ADCON0bits.GO = 1;

        //
        // Handle 1ms tasks
        //
        if (TimerHeartbeat)
        {
            TimerHeartbeat--;
        }
        // Every second, increment the last command times
        OneSecondCounter++;
        if (OneSecondCounter == ONE_SECOND_IN_MS)
        {
            OneSecondCounter = 0;
            if (LastCommandTime < 254)
            {
                LastCommandTime++;
            }

            // Check for a safety timeout
            if (SafetyTimeoutValue != 0x0000)
            {
                if (LastCommandTime > (SafetyTimeoutValue/1000))
                {
                    // Center all RC servo outputs to "0x80"
                    for (i=0; i < RC_SERVO_COUNT; i++)
                    {
                        RCServo_TargetWidth[i] = RCServo_Scale(0x80, i);
                        RCServo_Width[i] = RCServo_Scale(0x80, i);
                    }

                    // Turn off all DC motor outputs
                    for (i=0; i < 4; i++)
                    {
                        Motor_Value[i] = 0x00;
                        // Now update the motor's PWM hardware
                        PWMUpdateValue(i, 0x00);
                    }
                }
            }
        }
    }
    else
    {
        /* Unhandled interrupts */
        while (1)
        {
            ;
        }
    }
}
