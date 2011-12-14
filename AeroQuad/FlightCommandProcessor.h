/*
  AeroQuad v2.4 - April 2011
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

// FlightCommand.pde is responsible for decoding transmitter stick combinations
// for setting up AeroQuad modes such as motor arming and disarming

#ifndef _AQ_FLIGHT_COMMAND_READER_
#define _AQ_FLIGHT_COMMAND_READER_

void readPilotCommands() {
  
  readReceiver(); // Read quad configuration commands from transmitter when throttle down
  if (receiverCommand[THROTTLE] < MINCHECK) {
    zeroIntegralError();

    // Disarm motors (left stick lower left corner)
    if (receiverCommand[ZAXIS] < MINCHECK && armed == ON) {
      commandAllMotors(MINCOMMAND);
      digitalWrite(LED_Red,LOW);
      armed = OFF;
            
      #ifdef OSD
        notifyOSD(OSD_CENTER|OSD_WARN, "MOTORS UNARMED");
      #endif
      
      #if defined BattMonitorAutoDescent
        batteryMonitorAlarmCounter = 0;
        batteryMonitorStartThrottle = 0;
        batteyMonitorThrottleCorrection = 0.0;
      #endif
    }    
    
    // Zero Gyro and Accel sensors (left stick lower left, right stick lower right corner)
    if ((receiverCommand[ZAXIS] < MINCHECK) && (receiverCommand[XAXIS] > MAXCHECK) && (receiverCommand[YAXIS] < MINCHECK)) {
      calibrateGyro();
      computeAccelBias();
      storeSensorsZeroToEEPROM();
      calibrateKinematics();
      zeroIntegralError();
      pulseMotors(3);
    }   
    
    // Arm motors (left stick lower right corner)
    if (receiverCommand[ZAXIS] > MAXCHECK && armed == OFF && safetyCheck == ON) {
      zeroIntegralError();
      for (byte motor = 0; motor < LASTMOTOR; motor++) {
        motorCommand[motor] = MINTHROTTLE;
      }
      digitalWrite(LED_Red,HIGH);
      armed = ON;
    
      #ifdef OSD
        notifyOSD(OSD_CENTER|OSD_WARN, "!MOTORS ARMED!");
      #endif  
      
      
    }
    // Prevents accidental arming of motor output if no transmitter command received
    if (receiverCommand[ZAXIS] > MINCHECK) {
      safetyCheck = ON; 
    }
  }
  
  #ifdef RateModeOnly
    flightMode = ACRO;
  #else
    // Check Mode switch for Acro or Stable
    if (receiverCommand[MODE] > 1500) {
      flightMode = STABLE;
   }
    else {
      flightMode = ACRO;
    }
  #endif  
  
  #if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
   if (receiverCommand[AUX] < 1750) {
     if (altitudeHoldState != ALTPANIC ) {  // check for special condition with manditory override of Altitude hold
       if (isStoreAltitudeNeeded) {
         altitudeToHoldTarget = getAltitudeFromSensors();
         altitudeHoldThrottle = receiverCommand[THROTTLE];
         PID[ALTITUDE_HOLD_PID_IDX].integratedError = 0;
         PID[ALTITUDE_HOLD_PID_IDX].lastPosition = altitudeToHoldTarget;  // add to initialize hold position on switch turn on.
         isStoreAltitudeNeeded = false;
       }
       altitudeHoldState = ON;
     }
     // note, Panic will stay set until Althold is toggled off/on
   } 
   else {
     isStoreAltitudeNeeded = true;
     altitudeHoldState = OFF;
   }
  #endif
}

#endif // _AQ_FLIGHT_COMMAND_READER_


