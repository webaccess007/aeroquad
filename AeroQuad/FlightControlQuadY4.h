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

#ifndef _AQ_PROCESS_FLIGHT_CONTROL_Y4_MODE_H_
#define _AQ_PROCESS_FLIGHT_CONTROL_Y4_MODE_H_

/*
             UPPER/LOWER


       CW                  CCW
            
           0....Front....0  
           ......***......    
           ......***......
           ......***......    
           0....Back.....0  
      
                CW/CCW           
*/


#define FRONT_LEFT  MOTOR1
#define REAR        MOTOR2
#define FRONT_RIGHT MOTOR3
#define REAR_UNDER  MOTOR4
#define LASTMOTOR   MOTOR4+1

void applyMotorCommand() {
  // Front = Front/Right, Back = Left/Rear, Left = Front/Left, Right = Right/Rear 
  const int throttleCorrection = abs(motorAxisCommandYaw*2/4);
  motors->setMotorCommand(FRONT_LEFT,  (throttle)                    - motorAxisCommandPitch + motorAxisCommandRoll);
  motors->setMotorCommand(FRONT_RIGHT, (throttle)                    - motorAxisCommandPitch - motorAxisCommandRoll);
  motors->setMotorCommand(REAR_UNDER,  (throttle-throttleCorrection) + motorAxisCommandPitch - (YAW_DIRECTION * motorAxisCommandYaw));
  motors->setMotorCommand(REAR,        (throttle-throttleCorrection) + motorAxisCommandPitch + (YAW_DIRECTION * motorAxisCommandYaw));
}

void processMinMaxCommand() {
  
  if ((motors->getMotorCommand(FRONT_LEFT) <= MINTHROTTLE) || (motors->getMotorCommand(REAR_UNDER) <= MINTHROTTLE)){
    delta = receiver->getData(THROTTLE) - MINTHROTTLE;
    motorMaxCommand[FRONT_RIGHT] = constrain(receiver->getData(THROTTLE) + delta, minAcro, MAXCHECK);
    motorMaxCommand[REAR] =   constrain(receiver->getData(THROTTLE) + delta, minAcro, MAXCHECK);
  }
  else if ((motors->getMotorCommand(FRONT_LEFT) >= MAXCOMMAND) || (motors->getMotorCommand(REAR_UNDER) >= MAXCOMMAND)) {
    delta = MAXCOMMAND - receiver->getData(THROTTLE);
    motorMinCommand[FRONT_RIGHT] = constrain(receiver->getData(THROTTLE) - delta, minAcro, MAXCOMMAND);
    motorMinCommand[REAR]   = constrain(receiver->getData(THROTTLE) - delta, minAcro, MAXCOMMAND);
  }     
  else {
    motorMaxCommand[FRONT_RIGHT] = MAXCOMMAND;
    motorMaxCommand[REAR]      = MAXCOMMAND; 
    motorMinCommand[FRONT_RIGHT] = minAcro;
    motorMinCommand[REAR]      = minAcro;
  }

  if ((motors->getMotorCommand(REAR) <= MINTHROTTLE) || (motors->getMotorCommand(FRONT_RIGHT) <= MINTHROTTLE)){
    delta = receiver->getData(THROTTLE) - minAcro;
    motorMaxCommand[FRONT_LEFT] = constrain(receiver->getData(THROTTLE) + delta, minAcro, MAXCHECK);
    motorMaxCommand[REAR_UNDER] = constrain(receiver->getData(THROTTLE) + delta, minAcro, MAXCHECK);
  }
  else if ((motors->getMotorCommand(REAR) >= MAXCOMMAND) || (motors->getMotorCommand(FRONT_RIGHT) >= MAXCOMMAND)) {
    delta = MAXCOMMAND - receiver->getData(THROTTLE);
    motorMinCommand[FRONT_LEFT] = constrain(receiver->getData(THROTTLE) - delta, minAcro, MAXCOMMAND);
    motorMinCommand[REAR_UNDER] = constrain(receiver->getData(THROTTLE) - delta, minAcro, MAXCOMMAND);
  }     
  else {
    motorMaxCommand[FRONT_LEFT] = MAXCOMMAND;
    motorMaxCommand[REAR_UNDER] = MAXCOMMAND;
    motorMinCommand[FRONT_LEFT] = minAcro;
    motorMinCommand[REAR_UNDER] = minAcro;
  }
}

void processHardManuevers() {
  if (flightMode == ACRO) {
    if (receiver->getData(ROLL) < MINCHECK) {        // Maximum Left Roll Rate
      motorMinCommand[FRONT_RIGHT] =MAXCOMMAND;
      motorMinCommand[REAR] = MAXCOMMAND;
      motorMaxCommand[FRONT_LEFT] = minAcro;
      motorMaxCommand[REAR_UNDER]  = minAcro;
    }
    else if (receiver->getData(ROLL) > MAXCHECK) {   // Maximum Right Roll Rate
      motorMinCommand[FRONT_LEFT]  = MAXCOMMAND;
      motorMinCommand[REAR_UNDER]   = MAXCOMMAND;
      motorMaxCommand[FRONT_RIGHT] = minAcro;
      motorMaxCommand[REAR]  = minAcro;
    }
    else if (receiver->getData(PITCH) < MINCHECK) {  // Maximum Nose Up Pitch Rate
      motorMinCommand[FRONT_LEFT] =  MAXCOMMAND;
      motorMinCommand[FRONT_RIGHT] = MAXCOMMAND;
      motorMaxCommand[REAR_UNDER]   = minAcro;
      motorMaxCommand[REAR]  = minAcro;
    }
    else if (receiver->getData(PITCH) > MAXCHECK) {  // Maximum Nose Down Pitch Rate
      motorMinCommand[REAR_UNDER]   = MAXCOMMAND;
      motorMinCommand[REAR]  = MAXCOMMAND;
      motorMaxCommand[FRONT_LEFT]  = minAcro;
      motorMaxCommand[FRONT_RIGHT] = minAcro;
    }
  }
}

#endif // #define _AQ_PROCESS_FLIGHT_CONTROL_X_MODE_H_
