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

#ifndef _AQ_PROCESS_FLIGHT_CONTROL_OCTO_PLUS_MODE_H_
#define _AQ_PROCESS_FLIGHT_CONTROL_OCTO_PLUS_MODE_H_


/*  
             UPPER/LOWER


                 CW
       CCW                CCW
           0....Front....0  
           ......***......    
     CW    ......***......   CW
           ......***......    
           0....Back.....0  
       CCW                CCW
                 CW
*/     

#define FRONT       MOTOR1
#define FRONT_RIGHT MOTOR2
#define RIGHT       MOTOR3
#define REAR_RIGHT  MOTOR5
#define REAR        MOTOR4
#define REAR_LEFT   MOTOR6
#define LEFT        MOTOR7
#define FRONT_LEFT  MOTOR8
#define LASTMOTOR   MOTOR8+1

void applyMotorCommand() {
  // Front = Front/Right, Back = Left/Rear, Left = Front/Left, Right = Right/Rear 
  const int throttleCorrection  = abs(motorAxisCommandYaw*4/8);
  motorCommand[FRONT]       = (throttle-throttleCorrection) - motorAxisCommandPitch                                  - (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[FRONT_RIGHT] = (throttle-throttleCorrection) - motorAxisCommandPitch*7/10 - motorAxisCommandRoll*7/10 + (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[RIGHT]       = (throttle-throttleCorrection)                              - motorAxisCommandRoll      - (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[REAR_RIGHT]  = (throttle-throttleCorrection) + motorAxisCommandPitch*7/10 - motorAxisCommandRoll*7/10 + (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[REAR]        = (throttle-throttleCorrection) + motorAxisCommandPitch                                  - (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[REAR_LEFT]   = (throttle-throttleCorrection) + motorAxisCommandPitch*7/10 + motorAxisCommandRoll*7/10 + (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[LEFT]        = (throttle-throttleCorrection)                              + motorAxisCommandRoll      - (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[FRONT_LEFT]  = (throttle-throttleCorrection) - motorAxisCommandPitch*7/10 + motorAxisCommandRoll*7/10 + (YAW_DIRECTION * motorAxisCommandYaw);
}


// @Kenny, check this one for v3.0
void processMinMaxCommand() {
  
  if ((motorCommand[FRONT] <= MINTHROTTLE) || (motorCommand[REAR] <= MINTHROTTLE)){
    delta = receiverCommand[THROTTLE] - MINTHROTTLE;
    motorMaxCommand[RIGHT]       = constrain(receiverCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
    motorMaxCommand[FRONT_RIGHT] = constrain(receiverCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
    motorMaxCommand[REAR_RIGHT]  = constrain(receiverCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
    motorMaxCommand[LEFT]        = constrain(receiverCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
    motorMaxCommand[FRONT_LEFT]  = constrain(receiverCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
    motorMaxCommand[REAR_LEFT]   = constrain(receiverCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
  }
  else if ((motorCommand[FRONT] >= MAXCOMMAND) || (motorCommand[REAR] >= MAXCOMMAND)) {
    delta = MAXCOMMAND - receiverCommand[THROTTLE];
    motorMinCommand[RIGHT]       = constrain(receiverCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
    motorMinCommand[FRONT_RIGHT] = constrain(receiverCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
    motorMinCommand[REAR_RIGHT]  = constrain(receiverCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
    motorMinCommand[LEFT]        = constrain(receiverCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
    motorMinCommand[FRONT_LEFT]  = constrain(receiverCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
    motorMinCommand[REAR_LEFT]   = constrain(receiverCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
  }     
  else {
    motorMaxCommand[RIGHT]       = MAXCOMMAND;
    motorMaxCommand[FRONT_RIGHT] = MAXCOMMAND;
    motorMaxCommand[REAR_RIGHT]  = MAXCOMMAND;
    motorMaxCommand[LEFT]        = MAXCOMMAND;
    motorMaxCommand[FRONT_LEFT]  = MAXCOMMAND;
    motorMaxCommand[REAR_LEFT]   = MAXCOMMAND;
    motorMinCommand[RIGHT]       = MINTHROTTLE;
    motorMinCommand[FRONT_RIGHT] = MINTHROTTLE;
    motorMinCommand[REAR_RIGHT]  = MINTHROTTLE;
    motorMinCommand[LEFT]        = MINTHROTTLE;
    motorMinCommand[FRONT_LEFT]  = MINTHROTTLE;
    motorMinCommand[REAR_LEFT]   = MINTHROTTLE;
  }

  if ((motorCommand[LEFT] <= MINTHROTTLE) || (motorCommand[RIGHT] <= MINTHROTTLE)){
    delta = receiverCommand[THROTTLE] - MINTHROTTLE; 
    motorMaxCommand[FRONT]       = constrain(receiverCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
    motorMaxCommand[FRONT_LEFT]  = constrain(receiverCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
    motorMaxCommand[FRONT_RIGHT] = constrain(receiverCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
    motorMaxCommand[REAR]        = constrain(receiverCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
    motorMaxCommand[REAR_LEFT]   = constrain(receiverCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
    motorMaxCommand[REAR_RIGHT]  = constrain(receiverCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
  }
  else if ((motorCommand[LEFT] >= MAXCOMMAND) || (motorCommand[RIGHT] >= MAXCOMMAND)) {
    delta = MAXCOMMAND - receiverCommand[THROTTLE];
    motorMinCommand[FRONT]       = constrain(receiverCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
    motorMinCommand[FRONT_LEFT]  = constrain(receiverCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
    motorMinCommand[FRONT_RIGHT] = constrain(receiverCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
    motorMinCommand[REAR]        = constrain(receiverCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
    motorMinCommand[REAR_LEFT]   = constrain(receiverCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
    motorMinCommand[REAR_RIGHT]  = constrain(receiverCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
  }     
  else {
    motorMaxCommand[FRONT]       = MAXCOMMAND; 
    motorMaxCommand[FRONT_LEFT]  = MAXCOMMAND; 
    motorMaxCommand[FRONT_RIGHT] = MAXCOMMAND; 
    motorMaxCommand[REAR]        = MAXCOMMAND;
    motorMaxCommand[REAR_LEFT]   = MAXCOMMAND;
    motorMaxCommand[REAR_RIGHT]  = MAXCOMMAND;
    motorMinCommand[FRONT]       = MINTHROTTLE;
    motorMinCommand[FRONT_LEFT]  = MINTHROTTLE;
    motorMinCommand[FRONT_RIGHT] = MINTHROTTLE;
    motorMinCommand[REAR]        = MINTHROTTLE;
    motorMinCommand[REAR_LEFT]   = MINTHROTTLE;
    motorMinCommand[REAR_RIGHT]  = MINTHROTTLE;
  }
}

#endif // #define _AQ_PROCESS_FLIGHT_CONTROL_OCTO_PLUS_MODE_H_

