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

#ifndef _AQ_PROCESS_FLIGHT_CONTROL_HEX_Y6_MODE_H_
#define _AQ_PROCESS_FLIGHT_CONTROL_HEX_Y6_MODE_H_

#define LEFT            MOTOR1
#define REAR            MOTOR2
#define RIGHT           MOTOR3
#define REAR_UNDER      MOTOR4
#define LEFT_UNDER      MOTOR5
#define RIGHT_UNDER     MOTOR6
#define LASTMOTOR       MOTOR6+1

#define YAW_DIRECTION 1 // if you want to reverse the yaw correction direction
//#define YAW_DIRECTION -1


void applyMotorCommand() {
  // Front = Front/Right, Back = Left/Rear, Left = Front/Left, Right = Right/Rear 
  const int throttleCorrection = abs(motorAxisCommandYaw*2/6);
  motors->setMotorCommand(REAR,        (throttle - throttleCorrection)                        + (motorAxisCommandPitch*4/3) + YAW_DIRECTION * motorAxisCommandYaw);
  motors->setMotorCommand(RIGHT,       (throttle - throttleCorrection) - motorAxisCommandRoll - (motorAxisCommandPitch*2/3) + YAW_DIRECTION * motorAxisCommandYaw);  
  motors->setMotorCommand(LEFT,        (throttle - throttleCorrection) + motorAxisCommandRoll - (motorAxisCommandPitch*2/3) - YAW_DIRECTION * motorAxisCommandYaw);
  motors->setMotorCommand(REAR_UNDER,  (throttle - throttleCorrection)                        + (motorAxisCommandPitch*4/3) - YAW_DIRECTION * motorAxisCommandYaw);
  motors->setMotorCommand(RIGHT_UNDER, (throttle - throttleCorrection) - motorAxisCommandRoll - (motorAxisCommandPitch*2/3) - YAW_DIRECTION * motorAxisCommandYaw);
  motors->setMotorCommand(LEFT_UNDER,  (throttle - throttleCorrection) + motorAxisCommandRoll - (motorAxisCommandPitch*2/3) + YAW_DIRECTION * motorAxisCommandYaw);
}

void processMinMaxCommand() {
  
  if ((motors->getMotorCommand(LEFT) <= MINTHROTTLE) || (motors->getMotorCommand(REAR_UNDER) <= MINTHROTTLE)){
    delta = receiver->getData(THROTTLE) - MINTHROTTLE;
    motorMaxCommand[RIGHT] =       constrain(receiver->getData(THROTTLE) + delta, minAcro, MAXCHECK);
    motorMaxCommand[RIGHT_UNDER] = constrain(receiver->getData(THROTTLE) + delta, minAcro, MAXCHECK);
    motorMaxCommand[REAR] =        constrain(receiver->getData(THROTTLE) + delta, minAcro, MAXCHECK);
    motorMaxCommand[REAR_UNDER] =  constrain(receiver->getData(THROTTLE) + delta, minAcro, MAXCHECK);
  }
  else if ((motors->getMotorCommand(LEFT) >= MAXCOMMAND) || (motors->getMotorCommand(REAR_UNDER) >= MAXCOMMAND)) {
    delta = MAXCOMMAND - receiver->getData(THROTTLE);
    motorMinCommand[RIGHT]       = constrain(receiver->getData(THROTTLE) - delta, minAcro, MAXCOMMAND);
    motorMinCommand[RIGHT_UNDER] = constrain(receiver->getData(THROTTLE) - delta, minAcro, MAXCOMMAND);
    motorMinCommand[REAR]        = constrain(receiver->getData(THROTTLE) - delta, minAcro, MAXCOMMAND);
    motorMinCommand[REAR_UNDER]  = constrain(receiver->getData(THROTTLE) - delta, minAcro, MAXCOMMAND);
  }     
  else {
    motorMaxCommand[RIGHT]       = MAXCOMMAND;
    motorMaxCommand[RIGHT_UNDER] = MAXCOMMAND;
    motorMaxCommand[REAR]        = MAXCOMMAND; 
    motorMaxCommand[REAR_UNDER]  = MAXCOMMAND; 
    motorMinCommand[RIGHT]       = minAcro;
    motorMinCommand[RIGHT_UNDER] = minAcro;
    motorMinCommand[REAR]        = minAcro;
    motorMinCommand[REAR_UNDER]  = minAcro;
  }

  if ((motors->getMotorCommand(REAR) <= MINTHROTTLE) || (motors->getMotorCommand(RIGHT) <= MINTHROTTLE)){
    delta = receiver->getData(THROTTLE) - MINTHROTTLE;
    motorMaxCommand[LEFT]       = constrain(receiver->getData(THROTTLE) + delta, minAcro, MAXCHECK);
    motorMaxCommand[LEFT_UNDER] = constrain(receiver->getData(THROTTLE) + delta, minAcro, MAXCHECK);
    motorMaxCommand[REAR]       = constrain(receiver->getData(THROTTLE) + delta, minAcro, MAXCHECK);
    motorMaxCommand[REAR_UNDER] = constrain(receiver->getData(THROTTLE) + delta, minAcro, MAXCHECK);
  }
  else if ((motors->getMotorCommand(REAR) >= MAXCOMMAND) || (motors->getMotorCommand(RIGHT) >= MAXCOMMAND)) {
    delta = MAXCOMMAND - receiver->getData(THROTTLE);
    motorMinCommand[LEFT]       = constrain(receiver->getData(THROTTLE) - delta, minAcro, MAXCOMMAND);
    motorMinCommand[LEFT_UNDER] = constrain(receiver->getData(THROTTLE) - delta, minAcro, MAXCOMMAND);
    motorMinCommand[REAR]       = constrain(receiver->getData(THROTTLE) - delta, minAcro, MAXCOMMAND);
    motorMinCommand[REAR_UNDER] = constrain(receiver->getData(THROTTLE) - delta, minAcro, MAXCOMMAND);
  }     
  else {
    motorMaxCommand[LEFT]       = MAXCOMMAND;
    motorMaxCommand[LEFT_UNDER] = MAXCOMMAND;
    motorMaxCommand[REAR]       = MAXCOMMAND;
    motorMaxCommand[REAR_UNDER] = MAXCOMMAND;
    motorMinCommand[LEFT]       = minAcro;
    motorMinCommand[LEFT_UNDER] = minAcro;
    motorMinCommand[REAR]       = minAcro;
    motorMinCommand[REAR_UNDER] = minAcro;
  }
}

void processHardManuevers() {
//  if (flightMode == ACRO) {
//    if (receiver->getData(ROLL) < MINCHECK) {        // Maximum Left Roll Rate
//      motorMinCommand[RIGHT] =MAXCOMMAND;
//      motorMinCommand[REAR] = MAXCOMMAND;
//      motorMaxCommand[LEFT] = minAcro;
//      motorMaxCommand[REAR_UNDER]  = minAcro;
//    }
//    else if (receiver->getData(ROLL) > MAXCHECK) {   // Maximum Right Roll Rate
//      motorMinCommand[LEFT]  = MAXCOMMAND;
//      motorMinCommand[REAR_UNDER]   = MAXCOMMAND;
//      motorMaxCommand[RIGHT] = minAcro;
//      motorMaxCommand[REAR]  = minAcro;
//    }
//    else if (receiver->getData(PITCH) < MINCHECK) {  // Maximum Nose Up Pitch Rate
//      motorMinCommand[LEFT] =  MAXCOMMAND;
//      motorMinCommand[RIGHT] = MAXCOMMAND;
//      motorMaxCommand[REAR_UNDER]   = minAcro;
//      motorMaxCommand[REAR]  = minAcro;
//    }
//    else if (receiver->getData(PITCH) > MAXCHECK) {  // Maximum Nose Down Pitch Rate
//      motorMinCommand[REAR_UNDER]   = MAXCOMMAND;
//      motorMinCommand[REAR]  = MAXCOMMAND;
//      motorMaxCommand[LEFT]  = minAcro;
//      motorMaxCommand[RIGHT] = minAcro;
//    }
//  }
}

#endif // #define _AQ_PROCESS_FLIGHT_CONTROL_X_MODE_H_
