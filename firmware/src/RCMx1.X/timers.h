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

 /* 
 * File:   Timers.h
 * Author: brians
 *
 * Created on August 1, 2012, 1:32 PM
 */

#ifndef TIMERS_H
#define	TIMERS_H

#define TMR3_TICK_RATE          (16000000UL/8)
#define TMR3_MS_TO_TICKS(ms)    ((TMR3_TICK_RATE * ms)/1000)
#define RC_SERVO_FRAME_TIME     TMR3_MS_TO_TICKS(25)
// How much time between the leading edges of each of the 8 RC servo pins
#define RC_SERVO_NEXT_PIN_TIME  TMR3_MS_TO_TICKS(3)
#define ONE_SECOND_IN_MS        1000
#define SLOW_MOVE_MULT_FACTOR   4

void TimerInit(void);
UINT16 RCServo_Scale(UINT8 InputVal, UINT8 Channel);
UINT8 RCServo_Unscale(UINT16 OrigRCServo_Width, UINT8 Channel);
UINT16 RCServo_AccelScale(UINT8 InputVal);
UINT8 RCServo_AccelUnScale(UINT16 InputVal);

#endif	/* TIMERS_H */

