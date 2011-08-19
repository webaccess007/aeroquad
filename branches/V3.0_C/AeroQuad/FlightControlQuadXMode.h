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

#ifndef _AQ_PROCESS_FLIGHT_CONTROL_X_MODE_H_
#define _AQ_PROCESS_FLIGHT_CONTROL_X_MODE_H_

/*
       CW  0....Front....0 CCW
           ......***......    
           ......***......    
           ......***......    
      CCW  0....Back.....0  CW
*/


#define FRONT_LEFT  MOTOR1
#define REAR_RIGHT  MOTOR2
#define FRONT_RIGHT MOTOR3
#define REAR_LEFT   MOTOR4
#define LASTMOTOR   MOTOR4+1

void applyMotorCommand() {
  // Front = Front/Right, Back = Left/Rear, Left = Front/Left, Right = Right/Rear 
  const int throttleCorrection = abs(motorAxisCommandYaw*2/4);
  motorCommand[FRONT_LEFT] = (throttle-throttleCorrection) - motorAxisCommandPitch + motorAxisCommandRoll - (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[FRONT_RIGHT] = (throttle-throttleCorrection) - motorAxisCommandPitch - motorAxisCommandRoll + (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[REAR_LEFT] =   (throttle-throttleCorrection) + motorAxisCommandPitch + motorAxisCommandRoll + (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[REAR_RIGHT] = (throttle-throttleCorrection) + motorAxisCommandPitch - motorAxisCommandRoll - (YAW_DIRECTION * motorAxisCommandYaw);
}

void processMinMaxCommand() {
  if ((motorCommand[FRONT_LEFT] <= MINTHROTTLE) || (motorCommand[REAR_RIGHT] <= MINTHROTTLE)){
    delta = receiverData[THROTTLE] - MINTHROTTLE;
    motorMaxCommand[FRONT_RIGHT] = constrain(receiverData[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
    motorMaxCommand[REAR_LEFT] =   constrain(receiverData[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
  }
  else if ((motorCommand[FRONT_LEFT] >= MAXCOMMAND) || (motorCommand[REAR_RIGHT] >= MAXCOMMAND)) {
    delta = MAXCOMMAND - receiverData[THROTTLE];
    motorMinCommand[FRONT_RIGHT] = constrain(receiverData[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
    motorMinCommand[REAR_LEFT]   = constrain(receiverData[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
  }     
  else {
    motorMaxCommand[FRONT_RIGHT] = MAXCOMMAND;
    motorMaxCommand[REAR_LEFT]   = MAXCOMMAND; 
    motorMinCommand[FRONT_RIGHT] = MINTHROTTLE;
    motorMinCommand[REAR_LEFT]   = MINTHROTTLE;
  }

  if ((motorCommand[REAR_LEFT] <= MINTHROTTLE) || (motorCommand[FRONT_RIGHT] <= MINTHROTTLE)){
    delta = receiverData[THROTTLE] - MINTHROTTLE;
    motorMaxCommand[FRONT_LEFT] = constrain(receiverData[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
    motorMaxCommand[REAR_RIGHT] = constrain(receiverData[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
  }
  else if ((motorCommand[REAR_LEFT] >= MAXCOMMAND) || (motorCommand[FRONT_RIGHT] >= MAXCOMMAND)) {
    delta = MAXCOMMAND - receiverData[THROTTLE];
    motorMinCommand[FRONT_LEFT] = constrain(receiverData[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
    motorMinCommand[REAR_RIGHT] = constrain(receiverData[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
  }     
  else {
    motorMaxCommand[FRONT_LEFT] = MAXCOMMAND;
    motorMaxCommand[REAR_RIGHT] = MAXCOMMAND;
    motorMinCommand[FRONT_LEFT] = MINTHROTTLE;
    motorMinCommand[REAR_RIGHT] = MINTHROTTLE;
  }
}

void processHardManuevers() {
  if (flightMode == ACRO) {
    if (receiverData[ROLL] < MINCHECK) {        // Maximum Left Roll Rate
      motorMinCommand[FRONT_RIGHT] =MAXCOMMAND;
      motorMinCommand[REAR_RIGHT] = MAXCOMMAND;
      motorMaxCommand[FRONT_LEFT] = minAcro;
      motorMaxCommand[REAR_LEFT]  = minAcro;
    }
    else if (receiverData[ROLL] > MAXCHECK) {   // Maximum Right Roll Rate
      motorMinCommand[FRONT_LEFT]  = MAXCOMMAND;
      motorMinCommand[REAR_LEFT]   = MAXCOMMAND;
      motorMaxCommand[FRONT_RIGHT] = minAcro;
      motorMaxCommand[REAR_RIGHT]  = minAcro;
    }
    else if (receiverData[PITCH] < MINCHECK) {  // Maximum Nose Up Pitch Rate
      motorMinCommand[FRONT_LEFT] =  MAXCOMMAND;
      motorMinCommand[FRONT_RIGHT] = MAXCOMMAND;
      motorMaxCommand[REAR_LEFT]   = minAcro;
      motorMaxCommand[REAR_RIGHT]  = minAcro;
    }
    else if (receiverData[PITCH] > MAXCHECK) {  // Maximum Nose Down Pitch Rate
      motorMinCommand[REAR_LEFT]   = MAXCOMMAND;
      motorMinCommand[REAR_RIGHT]  = MAXCOMMAND;
      motorMaxCommand[FRONT_LEFT]  = minAcro;
      motorMaxCommand[FRONT_RIGHT] = minAcro;
    }
  }
}



#endif // #define _AQ_PROCESS_FLIGHT_CONTROL_X_MODE_H_

