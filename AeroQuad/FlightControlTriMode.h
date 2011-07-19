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

#ifndef _AQ_PROCESS_FLIGHT_CONTROL_TRI_MODE_H_
#define _AQ_PROCESS_FLIGHT_CONTROL_TRI_MODE_H_

#define SERVO       MOTOR1
#define REAR        MOTOR2
#define FRONT_RIGHT MOTOR3
#define FRONT_LEFT  MOTOR4
#define LASTMOTOR   MOTOR4+1

#define TRI_YAW_CONSTRAINT_MIN 1020
#define TRI_YAW_CONSTRAINT_MAX 2000
#define TRI_YAW_MIDDLE 1500
#define YAW_DIRECTION 1 // if you want to reverse the yaw correction direction
//#define YAW_DIRECTION -1

//unsigned long previousServoTime = 0;

void applyMotorCommand() {
  motors->setMotorCommand(FRONT_LEFT,  throttle + motorAxisCommandRoll - motorAxisCommandPitch*2/3);
  motors->setMotorCommand(FRONT_RIGHT, throttle - motorAxisCommandRoll - motorAxisCommandPitch*2/3);
  motors->setMotorCommand(REAR,   throttle + motorAxisCommandPitch*4/3);
  motors->setMotorCommand(SERVO,  constrain(TRI_YAW_MIDDLE + YAW_DIRECTION * motorAxisCommandYaw, TRI_YAW_CONSTRAINT_MIN, TRI_YAW_CONSTRAINT_MAX));
//  motors->setMotorCommand(SERVO,  filterSmooth(constrain(TRI_YAW_MIDDLE + YAW_DIRECTION * motorAxisCommandYaw, TRI_YAW_CONSTRAINT_MIN, TRI_YAW_CONSTRAINT_MAX),motors->getMotorCommand(SERVO),0.5));
}

void processMinMaxCommand() {
  if ((motors->getMotorCommand(FRONT_LEFT) <= MINTHROTTLE) || (motors->getMotorCommand(REAR) <= MINTHROTTLE)){
    delta = receiver->getData(THROTTLE) - MINTHROTTLE;
    motorMaxCommand[FRONT_RIGHT] = constrain(receiver->getData(THROTTLE) + delta, minAcro, MAXCHECK);
    motorMaxCommand[REAR] =   constrain(receiver->getData(THROTTLE) + delta, minAcro, MAXCHECK);
  }
  else if ((motors->getMotorCommand(FRONT_LEFT) >= MAXCOMMAND) || (motors->getMotorCommand(REAR) >= MAXCOMMAND)) {
    delta = MAXCOMMAND - receiver->getData(THROTTLE);
    motorMinCommand[FRONT_RIGHT] = constrain(receiver->getData(THROTTLE) - delta, minAcro, MAXCOMMAND);
    motorMinCommand[REAR]   = constrain(receiver->getData(THROTTLE) - delta, minAcro, MAXCOMMAND);
  }     
  else {
    motorMaxCommand[FRONT_RIGHT] = MAXCOMMAND;
    motorMaxCommand[REAR]        = MAXCOMMAND; 
    motorMinCommand[FRONT_RIGHT] = minAcro;
    motorMinCommand[REAR]        = minAcro;
  }

  if ((motors->getMotorCommand(REAR) <= MINTHROTTLE) || (motors->getMotorCommand(FRONT_RIGHT) <= MINTHROTTLE)){
    delta = receiver->getData(THROTTLE) - MINTHROTTLE;
    motorMaxCommand[FRONT_LEFT] = constrain(receiver->getData(THROTTLE) + delta, minAcro, MAXCHECK);
    motorMaxCommand[REAR] = constrain(receiver->getData(THROTTLE) + delta, minAcro, MAXCHECK);
  }
  else if ((motors->getMotorCommand(REAR) >= MAXCOMMAND) || (motors->getMotorCommand(FRONT_RIGHT) >= MAXCOMMAND)) {
    delta = MAXCOMMAND - receiver->getData(THROTTLE);
    motorMinCommand[FRONT_LEFT] = constrain(receiver->getData(THROTTLE) - delta, minAcro, MAXCOMMAND);
    motorMinCommand[REAR] = constrain(receiver->getData(THROTTLE) - delta, minAcro, MAXCOMMAND);
  }     
  else {
    motorMaxCommand[FRONT_LEFT] = MAXCOMMAND;
    motorMaxCommand[REAR]       = MAXCOMMAND;
    motorMinCommand[FRONT_LEFT] = minAcro;
    motorMinCommand[REAR]       = minAcro;
  }
  motorMaxCommand[SERVO] = MAXCOMMAND;
  motorMinCommand[SERVO] = MINCOMMAND;
  
}


void processHardManuevers() {
  if (flightMode == ACRO) {
    if (receiver->getData(ROLL) < MINCHECK) {        // Maximum Left Roll Rate
      motorMinCommand[FRONT_RIGHT] = MAXCOMMAND;
      motorMaxCommand[FRONT_LEFT]  = minAcro;
      motorMaxCommand[REAR]        = throttle + motorAxisCommandPitch*4/3;
    }
    else if (receiver->getData(ROLL) > MAXCHECK) {   // Maximum Right Roll Rate
      motorMinCommand[FRONT_LEFT]  = MAXCOMMAND;
      motorMaxCommand[FRONT_RIGHT] = minAcro;
      motorMaxCommand[REAR]        = throttle + motorAxisCommandPitch*4/3;
    }
    else if (receiver->getData(PITCH) < MINCHECK) {  // Maximum Nose Up Pitch Rate
      motorMinCommand[FRONT_LEFT]  = MAXCOMMAND;
      motorMinCommand[FRONT_RIGHT] = MAXCOMMAND;
      motorMaxCommand[REAR]        = minAcro;
    }
    else if (receiver->getData(PITCH) > MAXCHECK) {  // Maximum Nose Down Pitch Rate
      motorMinCommand[REAR]        = MAXCOMMAND;
      motorMaxCommand[FRONT_LEFT]  = minAcro;
      motorMaxCommand[FRONT_RIGHT] = minAcro;
    }
  }
}

#endif // #define _AQ_PROCESS_FLIGHT_CONTROL_X_MODE_H_
