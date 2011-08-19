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

#ifndef _AQ_PROCESS_FLIGHT_CONTROL_HEX_PLUS_MODE_H_
#define _AQ_PROCESS_FLIGHT_CONTROL_HEX_PLUS_MODE_H_

/*  
      @see here http://radio-commande.com/wp-content/uploads/2010/06/hex6.jpg

                 CCW
            
       CW  0....Front....0  CW
           ......***......    
           ......***......    
           ......***......    
      CCW  0....Back.....0  CCW
      
                 CW
*/     


#define FRONT_LEFT  MOTOR1
#define REAR_RIGHT  MOTOR2
#define FRONT_RIGHT MOTOR3
#define REAR_LEFT   MOTOR4
#define FRONT       MOTOR5
#define REAR        MOTOR6
#define LASTMOTOR   MOTOR6+1

void applyMotorCommand() {
  const int throttleCorrection = abs(motorAxisCommandYaw*3/6);
  motorCommand[FRONT_LEFT]  = (throttle - throttleCorrection) + motorAxisCommandRoll/2 - motorAxisCommandPitch/2 - (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[REAR_RIGHT]  = (throttle - throttleCorrection) - motorAxisCommandRoll/2 + motorAxisCommandPitch/2 + (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[FRONT_RIGHT] = (throttle - throttleCorrection) - motorAxisCommandRoll/2 - motorAxisCommandPitch/2 - (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[REAR_LEFT]   = (throttle - throttleCorrection) + motorAxisCommandRoll/2 + motorAxisCommandPitch/2 + (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[FRONT]       =  (throttle - throttleCorrection)                          - motorAxisCommandPitch   + (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[REAR]        =  (throttle - throttleCorrection)                          + motorAxisCommandPitch   - (YAW_DIRECTION * motorAxisCommandYaw);
}

void processMinMaxCommand() {
    if ((motorCommand[FRONT_LEFT] <= MINTHROTTLE) || (motorCommand[FRONT_RIGHT] <= MINTHROTTLE) || (motorCommand[REAR] <= MINTHROTTLE)) {
    delta = receiverData[THROTTLE] - MINTHROTTLE;
    motorMaxCommand[REAR_RIGHT] = constrain(receiverData[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
    motorMaxCommand[REAR_LEFT] =  constrain(receiverData[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
    motorMaxCommand[FRONT] =      constrain(receiverData[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
  }
  else if ((motorCommand[FRONT_LEFT] >= MAXCOMMAND) || (motorCommand[FRONT_RIGHT] >= MAXCOMMAND) || (motorCommand[REAR] >= MAXCOMMAND)) {
    delta = MAXCOMMAND - receiverData[THROTTLE];
    motorMinCommand[REAR_RIGHT] = constrain(receiverData[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
    motorMinCommand[REAR_LEFT] =  constrain(receiverData[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
    motorMinCommand[FRONT] =      constrain(receiverData[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
  }     
  else {
    motorMaxCommand[REAR_RIGHT] = MAXCOMMAND;
    motorMaxCommand[REAR_LEFT] =  MAXCOMMAND; 
    motorMaxCommand[FRONT] =      MAXCOMMAND; 
    motorMinCommand[REAR_RIGHT] = MINTHROTTLE;
    motorMinCommand[REAR_LEFT] =  MINTHROTTLE;
    motorMinCommand[FRONT] =      MINTHROTTLE;
  }

  if ((motorCommand[REAR_RIGHT] <= MINTHROTTLE) || (motorCommand[REAR_LEFT] <= MINTHROTTLE) || (motorCommand[FRONT] <= MINTHROTTLE)){
    delta = receiverData[THROTTLE] - MINTHROTTLE;
    motorMaxCommand[FRONT_LEFT] =  constrain(receiverData[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
    motorMaxCommand[FRONT_RIGHT] = constrain(receiverData[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
    motorMaxCommand[REAR] =        constrain(receiverData[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
  }
  else if ((motorCommand[REAR_RIGHT] >= MAXCOMMAND) || (motorCommand[REAR_LEFT] >= MAXCOMMAND) || (motorCommand[FRONT]  >= MAXCOMMAND)) {
    delta = MAXCOMMAND - receiverData[THROTTLE];
    motorMinCommand[FRONT_LEFT] =  constrain(receiverData[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
    motorMinCommand[FRONT_RIGHT] = constrain(receiverData[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
    motorMinCommand[REAR] =        constrain(receiverData[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
  }     
  else {
    motorMaxCommand[FRONT_LEFT] =  MAXCOMMAND;
    motorMaxCommand[FRONT_RIGHT] = MAXCOMMAND;
    motorMaxCommand[REAR] =        MAXCOMMAND;
    motorMinCommand[FRONT_LEFT] =  MINTHROTTLE;
    motorMinCommand[FRONT_RIGHT] = MINTHROTTLE;
    motorMinCommand[REAR] =        MINTHROTTLE;
  }
}

void processHardManuevers() {
  if (flightMode == ACRO) {
    if (receiverData[ROLL] < MINCHECK) {        // Maximum Left Roll Rate
      motorMinCommand[FRONT_RIGHT] = MAXCOMMAND;
      motorMinCommand[REAR_RIGHT] =  MAXCOMMAND;
      motorMaxCommand[FRONT_LEFT] =  minAcro;
      motorMaxCommand[REAR_LEFT] =   minAcro;
    }
    else if (receiverData[ROLL] > MAXCHECK) {   // Maximum Right Roll Rate
      motorMinCommand[FRONT_LEFT] =  MAXCOMMAND;
      motorMinCommand[REAR_LEFT] =   MAXCOMMAND;
      motorMaxCommand[FRONT_RIGHT] = minAcro;
      motorMaxCommand[REAR_RIGHT] =  minAcro;
    }
    else if (receiverData[PITCH] < MINCHECK) {  // Maximum Nose Up Pitch Rate
      motorMinCommand[FRONT] =       MAXCOMMAND;
      motorMinCommand[FRONT_LEFT] =  MAXCOMMAND;
      motorMinCommand[FRONT_RIGHT] = MAXCOMMAND;
      motorMaxCommand[REAR] =        minAcro;
      motorMaxCommand[REAR_LEFT] =   minAcro;
      motorMaxCommand[REAR_RIGHT] =  minAcro;
    }
    else if (receiverData[PITCH] > MAXCHECK) {  // Maximum Nose Down Pitch Rate
      motorMinCommand[REAR] =        MAXCOMMAND;
      motorMinCommand[REAR_LEFT] =   MAXCOMMAND;
      motorMinCommand[REAR_RIGHT] =  MAXCOMMAND;
      motorMaxCommand[FRONT] =       minAcro;
      motorMaxCommand[FRONT_LEFT] =  minAcro;
      motorMaxCommand[FRONT_RIGHT] = minAcro;
    }
  }
}


#endif  // #define _AQ_PROCESS_FLIGHT_CONTROL_HEX_PLUS_MODE_H_

