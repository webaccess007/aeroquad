/*
  AeroQuad v3.0 - April 2011
  www.AeroQuad.com 
  Copyright (c) 2011 Ted Carancho.  All rights reserved.
  An Open Source Arduino based multicopter.
 
  This program is free software: you can redistribute it and/or modify 
  it under the terms of the GNU General Public License as published by 
  the Free Software Foundation, either version 3 of the License, or 
  (at your option) any later version. 

  This program is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
  GNU General Public License for more details. 

  You should have received a copy of the GNU General Public License 
  along with this program. If not, see <http://www.gnu.org/licenses/>. 
*/


#ifndef _AEROQUAD_MOTORS_PWM_328P_H_
#define _AEROQUAD_MOTORS_PWM_328P_H_

#include "Motors.h"

/*  Motor  328 Pin Port
    0        3     PD3/OC2B
    1        9     PB1/OC1A
    2       10     PB2/OC1B
    3       11     PB3/OC2A
*/

#define PWM_FREQUENCY 244   // in Hz
#define PWM_PRESCALER 256
#define PWM_COUNTER_PERIOD (F_CPU/PWM_PRESCALER/PWM_FREQUENCY)

void commandAllMotors(int _motorCommand) {                  // Sends commands to all motors
  OCR2B = _motorCommand / 16 ;
  OCR1A = _motorCommand / 16 ;
  OCR1B = _motorCommand / 16 ;
  OCR2A = _motorCommand / 16 ;
}

void initializeMotors(NB_Motors numbers) {

  numberOfMotors = numbers;
  
  DDRB = DDRB | B00001110;                                  // Set ports to output PB1-3
  DDRD = DDRD | B00001000;                                  // Set port to output PD3

  commandAllMotors(1000);                                   // Initialize motors to 1000us (stopped)

  // Init PWM Timer 1  16 bit
  TCCR1A = (1<<WGM11)|(1<<COM1A1)|(1<<COM1B1);
  TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS12);
  ICR1 = PWM_COUNTER_PERIOD;
  // Init PWM Timer 2   8bit                               // WGMn1 WGMn2 = Mode ? Fast PWM, TOP = 0xFF ,Update of OCRnx at BOTTOM
  TCCR2A = (1<<WGM20)|(1<<WGM21)|(1<<COM2A1)|(1<<COM2B1);  // Clear OCnA/OCnB on compare match, set OCnA/OCnB at BOTTOM (non-inverting mode)
  TCCR2B = (1<<CS22)|(1<<CS21);                            // Prescaler set to 256, that gives us a resolution of 16us
  // TOP is fixed at 255                                   // Output_PWM_Frequency = 244hz = 16000000/(256*(1+255)) = Clock_Speed / (Prescaler * (1 + TOP))
}

void writeMotors() {
  OCR2B = motorCommand[0] / 16;
  OCR1A = motorCommand[1] / 16;
  OCR1B = motorCommand[2] / 16;
  OCR2A = motorCommand[3] / 16;
}

#endif

