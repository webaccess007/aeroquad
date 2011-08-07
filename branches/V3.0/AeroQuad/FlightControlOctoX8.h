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

#ifndef _AQ_PROCESS_FLIGHT_CONTROL_OCTO_X8_MODE_H_
#define _AQ_PROCESS_FLIGHT_CONTROL_OCTO_X8_MODE_H_

#define FRONT_LEFT    MOTOR1
#define REAR_RIGHT    MOTOR2
#define FRONT_RIGHT   MOTOR3
#define REAR_LEFT     MOTOR4
#define FRONT_LEFT_2  MOTOR5
#define REAR_RIGHT_2  MOTOR6
#define FRONT_RIGHT_2 MOTOR7
#define REAR_LEFT_2   MOTOR8
#define LASTMOTOR     MOTOR8+1

void applyMotorCommand() {
  // Front = Front/Right, Back = Left/Rear, Left = Front/Left, Right = Right/Rear 
  const int throttleCorrection = abs(motorAxisCommandYaw*4/8);
  motors->setMotorCommand(FRONT_LEFT,    (throttle-throttleCorrection) - motorAxisCommandPitch + motorAxisCommandRoll - motorAxisCommandYaw);
  motors->setMotorCommand(FRONT_RIGHT,   (throttle-throttleCorrection) - motorAxisCommandPitch - motorAxisCommandRoll + motorAxisCommandYaw);
  motors->setMotorCommand(REAR_LEFT,     (throttle-throttleCorrection) + motorAxisCommandPitch + motorAxisCommandRoll + motorAxisCommandYaw);
  motors->setMotorCommand(REAR_RIGHT,    (throttle-throttleCorrection) + motorAxisCommandPitch - motorAxisCommandRoll - motorAxisCommandYaw);
  motors->setMotorCommand(FRONT_LEFT_2,  (throttle-throttleCorrection) - motorAxisCommandPitch + motorAxisCommandRoll + motorAxisCommandYaw);
  motors->setMotorCommand(FRONT_RIGHT_2, (throttle-throttleCorrection) - motorAxisCommandPitch - motorAxisCommandRoll - motorAxisCommandYaw);
  motors->setMotorCommand(REAR_LEFT_2,   (throttle-throttleCorrection) + motorAxisCommandPitch + motorAxisCommandRoll - motorAxisCommandYaw);
  motors->setMotorCommand(REAR_RIGHT_2,  (throttle-throttleCorrection) + motorAxisCommandPitch - motorAxisCommandRoll + motorAxisCommandYaw);
}

void processMinMaxCommand() {
  if ((motors->getMotorCommand(FRONT_LEFT) <= MINTHROTTLE) || (motors->getMotorCommand(REAR_RIGHT) <= MINTHROTTLE)){
    delta = receiver->getData(THROTTLE) - MINTHROTTLE;
    motorMaxCommand[FRONT_RIGHT] =   constrain(receiver->getData(THROTTLE) + delta, minAcro, MAXCHECK);
    motorMaxCommand[REAR_LEFT] =     constrain(receiver->getData(THROTTLE) + delta, minAcro, MAXCHECK);
    motorMaxCommand[FRONT_RIGHT_2] = constrain(receiver->getData(THROTTLE) + delta, minAcro, MAXCHECK);
    motorMaxCommand[REAR_LEFT_2] =   constrain(receiver->getData(THROTTLE) + delta, minAcro, MAXCHECK);
  }
  else if ((motors->getMotorCommand(FRONT_LEFT) >= MAXCOMMAND) || (motors->getMotorCommand(REAR_RIGHT) >= MAXCOMMAND)) {
    delta = MAXCOMMAND - receiver->getData(THROTTLE);
    motorMinCommand[FRONT_RIGHT]   = constrain(receiver->getData(THROTTLE) - delta, minAcro, MAXCOMMAND);
    motorMinCommand[REAR_LEFT]     = constrain(receiver->getData(THROTTLE) - delta, minAcro, MAXCOMMAND);
    motorMinCommand[FRONT_RIGHT_2] = constrain(receiver->getData(THROTTLE) - delta, minAcro, MAXCOMMAND);
    motorMinCommand[REAR_LEFT_2]   = constrain(receiver->getData(THROTTLE) - delta, minAcro, MAXCOMMAND);
  }     
  else {
    motorMaxCommand[FRONT_RIGHT]   = MAXCOMMAND;
    motorMaxCommand[REAR_LEFT]     = MAXCOMMAND; 
    motorMaxCommand[FRONT_RIGHT_2] = MAXCOMMAND;
    motorMaxCommand[REAR_LEFT_2]   = MAXCOMMAND; 
    motorMinCommand[FRONT_RIGHT]   = minAcro;
    motorMinCommand[REAR_LEFT]     = minAcro;
    motorMinCommand[FRONT_RIGHT_2] = minAcro;
    motorMinCommand[REAR_LEFT_2]   = minAcro;
  }

  if ((motors->getMotorCommand(REAR_LEFT) <= MINTHROTTLE) || (motors->getMotorCommand(FRONT_RIGHT) <= MINTHROTTLE)){
    delta = receiver->getData(THROTTLE) - MINTHROTTLE;
    motorMaxCommand[FRONT_LEFT]   = constrain(receiver->getData(THROTTLE) + delta, minAcro, MAXCHECK);
    motorMaxCommand[REAR_RIGHT]   = constrain(receiver->getData(THROTTLE) + delta, minAcro, MAXCHECK);
    motorMaxCommand[FRONT_LEFT_2] = constrain(receiver->getData(THROTTLE) + delta, minAcro, MAXCHECK);
    motorMaxCommand[REAR_RIGHT_2] = constrain(receiver->getData(THROTTLE) + delta, minAcro, MAXCHECK);
  }
  else if ((motors->getMotorCommand(REAR_LEFT) >= MAXCOMMAND) || (motors->getMotorCommand(FRONT_RIGHT) >= MAXCOMMAND)) {
    delta = MAXCOMMAND - receiver->getData(THROTTLE);
    motorMinCommand[FRONT_LEFT]   = constrain(receiver->getData(THROTTLE) - delta, minAcro, MAXCOMMAND);
    motorMinCommand[REAR_RIGHT]   = constrain(receiver->getData(THROTTLE) - delta, minAcro, MAXCOMMAND);
    motorMinCommand[FRONT_LEFT_2] = constrain(receiver->getData(THROTTLE) - delta, minAcro, MAXCOMMAND);
    motorMinCommand[REAR_RIGHT_2] = constrain(receiver->getData(THROTTLE) - delta, minAcro, MAXCOMMAND);
  }     
  else {
    motorMaxCommand[FRONT_LEFT]   = MAXCOMMAND;
    motorMaxCommand[REAR_RIGHT]   = MAXCOMMAND;
    motorMaxCommand[FRONT_LEFT_2] = MAXCOMMAND;
    motorMaxCommand[REAR_RIGHT_2] = MAXCOMMAND;
    motorMinCommand[FRONT_LEFT]   = minAcro;
    motorMinCommand[REAR_RIGHT]   = minAcro;
    motorMinCommand[FRONT_LEFT_2] = minAcro;
    motorMinCommand[REAR_RIGHT_2] = minAcro;
  }
}

void processHardManuevers() {
  if (flightMode == ACRO) {
    if (receiver->getData(ROLL) < MINCHECK) {        // Maximum Left Roll Rate
      motorMinCommand[FRONT_RIGHT]   = MAXCOMMAND;
      motorMinCommand[REAR_RIGHT]    = MAXCOMMAND;
      motorMinCommand[FRONT_RIGHT_2] = MAXCOMMAND;
      motorMinCommand[REAR_RIGHT_2]  = MAXCOMMAND;
      motorMaxCommand[FRONT_LEFT]    = minAcro;
      motorMaxCommand[REAR_LEFT]     = minAcro;
      motorMaxCommand[FRONT_LEFT_2]  = minAcro;
      motorMaxCommand[REAR_LEFT_2]   = minAcro;
    }
    else if (receiver->getData(ROLL) > MAXCHECK) {   // Maximum Right Roll Rate
      motorMinCommand[FRONT_LEFT]    = MAXCOMMAND;
      motorMinCommand[REAR_LEFT]     = MAXCOMMAND;
      motorMinCommand[FRONT_LEFT_2]  = MAXCOMMAND;
      motorMinCommand[REAR_LEFT_2]   = MAXCOMMAND;
      motorMaxCommand[FRONT_RIGHT]   = minAcro;
      motorMaxCommand[REAR_RIGHT]    = minAcro;
      motorMaxCommand[FRONT_RIGHT_2] = minAcro;
      motorMaxCommand[REAR_RIGHT_2]  = minAcro;
    }
    else if (receiver->getData(PITCH) < MINCHECK) {  // Maximum Nose Up Pitch Rate
      motorMinCommand[FRONT_LEFT]    =  MAXCOMMAND;
      motorMinCommand[FRONT_RIGHT]   = MAXCOMMAND;
      motorMinCommand[FRONT_LEFT_2]  =  MAXCOMMAND;
      motorMinCommand[FRONT_RIGHT_2] = MAXCOMMAND;
      motorMaxCommand[REAR_LEFT]     = minAcro;
      motorMaxCommand[REAR_RIGHT]    = minAcro;
      motorMaxCommand[REAR_LEFT_2]   = minAcro;
      motorMaxCommand[REAR_RIGHT_2]  = minAcro;
    }
    else if (receiver->getData(PITCH) > MAXCHECK) {  // Maximum Nose Down Pitch Rate
      motorMinCommand[REAR_LEFT]     = MAXCOMMAND;
      motorMinCommand[REAR_RIGHT]    = MAXCOMMAND;
      motorMinCommand[REAR_LEFT_2]   = MAXCOMMAND;
      motorMinCommand[REAR_RIGHT_2]  = MAXCOMMAND;
      motorMaxCommand[FRONT_LEFT]    = minAcro;
      motorMaxCommand[FRONT_RIGHT]   = minAcro;
      motorMaxCommand[FRONT_LEFT_2]  = minAcro;
      motorMaxCommand[FRONT_RIGHT_2] = minAcro;
    }
  }
}



#endif // #define _AQ_PROCESS_FLIGHT_CONTROL_X_MODE_H_

