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

// FlightControl.pde is responsible for combining sensor measurements and
// transmitter commands into motor commands for the defined flight configuration (X, +, etc.)
//////////////////////////////////////////////////////////////////////////////
/////////////////////////// calculateFlightError /////////////////////////////
//////////////////////////////////////////////////////////////////////////////

#ifndef _AQ_PROCESS_FLIGHT_CONTROL_H_
#define _AQ_PROCESS_FLIGHT_CONTROL_H_

#define ATTITUDE_SCALING (0.75 * PWM2RAD)

void calculateFlightError()
{
  if (flightMode == ATTITUDE_FLIGHT_MODE) {
    float rollAttitudeCmd  = updatePID((receiverCommand[XAXIS] - receiverZero[XAXIS]) * ATTITUDE_SCALING, kinematicsAngle[XAXIS], &PID[ATTITUDE_XAXIS_PID_IDX]);
    float pitchAttitudeCmd = updatePID((receiverCommand[YAXIS] - receiverZero[YAXIS]) * ATTITUDE_SCALING, -kinematicsAngle[YAXIS], &PID[ATTITUDE_YAXIS_PID_IDX]);
    motorAxisCommandRoll   = updatePID(rollAttitudeCmd, correctedRateVector[XAXIS], &PID[ATTITUDE_GYRO_XAXIS_PID_IDX]);
    motorAxisCommandPitch  = updatePID(pitchAttitudeCmd, -correctedRateVector[YAXIS], &PID[ATTITUDE_GYRO_YAXIS_PID_IDX]);
  }
  else {
    motorAxisCommandRoll = updatePID(getReceiverSIData(XAXIS), correctedRateVector[XAXIS], &PID[RATE_XAXIS_PID_IDX]);
    motorAxisCommandPitch = updatePID(getReceiverSIData(YAXIS), -correctedRateVector[YAXIS], &PID[RATE_YAXIS_PID_IDX]);
  }
}

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// processCalibrateESC //////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void processCalibrateESC()
{
  switch (calibrateESC) { // used for calibrating ESC's
  case 1:
    for (byte motor = 0; motor < LASTMOTOR; motor++)
      motorCommand[motor] = MAXCOMMAND;
    break;
  case 3:
    for (byte motor = 0; motor < LASTMOTOR; motor++)
      motorCommand[motor] = constrain(testCommand, 1000, 1200);
    break;
  case 5:
    for (byte motor = 0; motor < LASTMOTOR; motor++)
      motorCommand[motor] = constrain(motorConfiguratorCommand[motor], 1000, 1200);
    safetyCheck = ON;
    break;
  default:
    for (byte motor = 0; motor < LASTMOTOR; motor++)
      motorCommand[motor] = MINCOMMAND;
  }
  // Send calibration commands to motors
  writeMotors(); // Defined in Motors.h
}

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// processHeadingHold ///////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void processHeading()
{
  if (headingHoldConfig == ON) {

    #if defined(HeadingMagHold)
      heading = degrees(kinematicsAngle[ZAXIS]);
    #else
      heading = degrees(gyroHeading);
    #endif

    // Always center relative heading around absolute heading chosen during yaw command
    // This assumes that an incorrect yaw can't be forced on the AeroQuad >180 or <-180 degrees
    // This is done so that AeroQuad does not accidentally hit transition between 0 and 360 or -180 and 180
    // AKA - THERE IS A BUG HERE - if relative heading is greater than 180 degrees, the PID will swing from negative to positive
    // Doubt that will happen as it would have to be uncommanded.
    relativeHeading = heading - setHeading;
    if (heading <= (setHeading - 180)) {
      relativeHeading += 360;
    }
    if (heading >= (setHeading + 180)) {
      relativeHeading -= 360;
    }

    // Apply heading hold only when throttle high enough to start flight
    if (receiverCommand[THROTTLE] > MINCHECK ) { 
      
      if ((receiverCommand[ZAXIS] > (MIDCOMMAND + 25)) || (receiverCommand[ZAXIS] < (MIDCOMMAND - 25))) {
        
        // If commanding yaw, turn off heading hold and store latest heading
        setHeading = heading;
        headingHold = 0;
        PID[HEADING_HOLD_PID_IDX].integratedError = 0;
        headingHoldState = OFF;
        headingTime = currentTime;
      }
      else {
        if (relativeHeading < 0.25 && relativeHeading > -0.25) {
          headingHold = 0;
          PID[HEADING_HOLD_PID_IDX].integratedError = 0;
        }
        else if (headingHoldState == OFF) { // quick fix to soften heading hold on new heading
          if ((currentTime - headingTime) > 500000) {
            headingHoldState = ON;
            headingTime = currentTime;
            setHeading = heading;
            headingHold = 0;
          }
        }
        else {
        // No new yaw input, calculate current heading vs. desired heading heading hold
        // Relative heading is always centered around zero
          headingHold = updatePID(0, relativeHeading, &PID[HEADING_HOLD_PID_IDX]);
          headingTime = currentTime; // quick fix to soften heading hold, wait 100ms before applying heading hold
        }
      }
    }
    else {
      // minimum throttle not reached, use off settings
      setHeading = heading;
      headingHold = 0;
      PID[HEADING_HOLD_PID_IDX].integratedError = 0;
    }
  }
  // NEW SI Version
  commandedYaw = constrain(getReceiverSIData(ZAXIS) + radians(headingHold), -PI, PI);
  motorAxisCommandYaw = updatePID(commandedYaw, gyroRate[ZAXIS], &PID[ZAXIS_PID_IDX]);
}




#if defined BattMonitorAutoDescent
  void processBatteryMonitorThrottleAdjustment() {
    
    if (batteryMonitorAlarmCounter < BATTERY_MONITOR_MAX_ALARM_COUNT) {
      if (batteryAlarm) {
        batteryMonitorAlarmCounter++;
      }
    }
    else {
      #if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
        if (altitudeHoldState == ON) {
          altitudeToHoldTarget -= 0.005;
        }
        else {
      #endif
          if (batteryMonitorStartThrottle == 0) {  // init battery monitor throttle correction!
            batteryMonitorStartTime = millis();
            if (throttle < batteryMonitorThrottleTarget) {
              batteryMonitorStartThrottle = batteryMonitorThrottleTarget;
            }
            else {
              batteryMonitorStartThrottle = throttle; 
            }
          }
          int batteryMonitorThrottle = map(millis()-batteryMonitorStartTime, 0, batteryMonitorGoinDownTime, batteryMonitorStartThrottle, batteryMonitorThrottleTarget);
          if (batteryMonitorThrottle < batteryMonitorThrottleTarget) {
            batteryMonitorThrottle = batteryMonitorThrottleTarget;
          }
          if (throttle < batteryMonitorThrottle) {
            batteyMonitorThrottleCorrection = 0;
          }
          else {
            batteyMonitorThrottleCorrection = batteryMonitorThrottle - throttle;
          }
      #if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
        }
      #endif
    }
  }
#endif  


void processThrottleCorrection() {
 
  // Thank Ziojo for this little adjustment on throttle when manuevering!
  int throttleAsjust = throttle / (cos (radians (kinematicsAngle[XAXIS])) * cos (radians (kinematicsAngle[YAXIS])));
  throttleAsjust = constrain ((throttleAsjust - throttle), 0, 160); //compensate max  +/- 25 deg XAXIS or YAXIS or  +/- 18 ( 18(XAXIS) + 18(YAXIS))
  throttle = throttle + throttleAsjust + (int)batteyMonitorThrottleCorrection;
}



void processHardManuevers() {
  
  if ((receiverCommand[XAXIS] < MINCHECK) ||
      (receiverCommand[XAXIS] > MAXCHECK) ||
      (receiverCommand[YAXIS] < MINCHECK) ||
      (receiverCommand[YAXIS] > MAXCHECK)) {  
        
    for (int motor = 0; motor < LASTMOTOR; motor++) {
      motorMinCommand[motor] = minAcro;
      motorMaxCommand[motor] = MAXCOMMAND;
    }
  }
}


//////////////////////////////////////////////////////////////////////////////
/////////////////////////// processMinMaxCommand ////////////////////////
//////////////////////////////////////////////////////////////////////////////
void processMinMaxCommand()
{
  for (byte motor = 0; motor < LASTMOTOR; motor++)
  {
    motorMinCommand[motor] = MINTHROTTLE;
    motorMaxCommand[motor] = MAXCOMMAND;
  }

  int maxMotor = motorCommand[0];
  
  for (byte motor=1; motor < LASTMOTOR; motor++) {
    if (motorCommand[motor] > maxMotor) {
      maxMotor = motorCommand[motor];
    }
  }
    
  for (byte motor = 0; motor < LASTMOTOR; motor++) {
    if (maxMotor > MAXCHECK) {
      motorCommand[motor] =  motorCommand[motor] - (maxMotor - MAXCHECK);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// processFlightControl main function ///////////////
//////////////////////////////////////////////////////////////////////////////
void processFlightControl() {
  
  // ********************** Calculate Flight Error ***************************
  calculateFlightError();
  
  // ********************** Update Yaw ***************************************
  processHeading();
  
  if (frameCounter %  10 == 0) {  //   10 Hz tasks
    // ********************** Process Altitude hold **************************
    processAltitudeHold();
    // ********************** Process Battery monitor hold **************************
    #if defined BattMonitorAutoDescent
      processBatteryMonitorThrottleAdjustment();
    #endif
    // ********************** Process throttle correction ********************
    processThrottleCorrection();
  }

  // ********************** Calculate Motor Commands *************************
  if (armed && safetyCheck) {
    applyMotorCommand();
  } 

  // *********************** process min max motor command *******************
  if (receiverCommand[THROTTLE] <= MAXCHECK) { // if the throttle is about the max, we used true PID values! PATCH for max throttle bug
    processMinMaxCommand();
  }

  // Allows quad to do acrobatics by lowering power to opposite motors during hard manuevers
  if (flightMode == RATE_FLIGHT_MODE) {
    processHardManuevers();    // This is not a good way to handle loop, just learn to pilot and do it normally!
  }
  
  // Apply limits to motor commands
  for (byte motor = 0; motor < LASTMOTOR; motor++) {
    motorCommand[motor] = constrain(motorCommand[motor], motorMinCommand[motor], motorMaxCommand[motor]);
  }

  // If throttle in minimum position, don't apply yaw
  if (receiverCommand[THROTTLE] < MINCHECK) {
    for (byte motor = 0; motor < LASTMOTOR; motor++) {
      motorCommand[motor] = MINTHROTTLE;
    }
  }

  // ESC Calibration
  if (armed == OFF) {
    processCalibrateESC();
  }

  // *********************** Command Motors **********************
  if (armed == ON && safetyCheck == ON) {
    writeMotors();
  }
}

#endif //#define _AQ_PROCESS_FLIGHT_CONTROL_H_

