/*
  AeroQuad v2.5 Beta 1 - July 2011
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
#define ATTITUDE_SCALING (0.75 * PWM2RAD)
void calculateFlightError(void)
{
  if (flightMode == ACRO) {
    motorAxisCommand[ROLL]  = updatePID(receiverData[ROLL], gyro.value[ROLL], &PID[ROLL]);
    motorAxisCommand[PITCH] = updatePID(receiverData[PITCH], -gyro.value[PITCH], &PID[PITCH]);
  }
  else {
    float rollAttitudeCmd  = updatePID(receiverData[ROLL]  * ATTITUDE_SCALING,  angle.value[ROLL],  &PID[LEVELROLL]);
    float pitchAttitudeCmd = updatePID(receiverData[PITCH] * ATTITUDE_SCALING, -angle.value[PITCH], &PID[LEVELPITCH]);
    motorAxisCommand[ROLL]  = updatePID(rollAttitudeCmd,   gyro.value[ROLL],  &PID[LEVELGYROROLL]);
    motorAxisCommand[PITCH] = updatePID(pitchAttitudeCmd, -gyro.value[PITCH], &PID[LEVELGYROPITCH]);
  }
}

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// processCalibrateESC //////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void processCalibrateESC(void)
{
  switch (calibrateESC) { // used for calibrating ESC's
  case 1:
    for (byte motor = FIRSTMOTOR; motor < LASTMOTOR; motor++)
      motorCommand[motor] = MAXCOMMAND;
    break;
  case 3:
    for (byte motor = FIRSTMOTOR; motor < LASTMOTOR; motor++)
      motorCommand[motor] = constrain(testCommand, 1000, 1200);
    break;
  case 5:
    for (byte motor = FIRSTMOTOR; motor < LASTMOTOR; motor++)
      motorCommand[motor] = constrain(remoteMotorCommand[motor], 1000, 1200);
    safetyCheck = ON;
    break;
  default:
    for (byte motor = FIRSTMOTOR; motor < LASTMOTOR; motor++)
      motorCommand[motor] = MINCOMMAND;
  }
  // Send calibration commands to motors
  writeMotors(); // Defined in Motors.h
}

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// processHeadingHold ///////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void processHeading(void)
{
  if (headingHoldConfig == ON) {

    #if defined(HeadingMagHold) || defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
      heading = degrees(flightAngle->getHeading(YAW));
    #else
      heading = degrees(angle.value[YAW]);
    #endif

    // Always center relative heading around absolute heading chosen during yaw command
    // This assumes that an incorrect yaw can't be forced on the AeroQuad >180 or <-180 degrees
    // This is done so that AeroQuad does not accidentally hit transition between 0 and 360 or -180 and 180
    // AKA - THERE IS A BUG HERE - if relative heading is greater than 180 degrees, the PID will swing from negative to positive
    // Doubt that will happen as it would have to be uncommanded.
    relativeHeading = heading - setHeading;
    if (heading <= (setHeading - 180)) relativeHeading += 360;
    if (heading >= (setHeading + 180)) relativeHeading -= 360;

    // Apply heading hold only when throttle high enough to start flight
    if (receiverData[THROTTLE] > MINCHECK ) { 
      if ((receiverData[YAW] > (MIDCOMMAND + 25)) || (receiverData[YAW] < (MIDCOMMAND - 25))) {
        // If commanding yaw, turn off heading hold and store latest heading
        setHeading = heading;
        headingHold = 0;
        PID[HEADING].integratedError = 0;
        headingHoldState = OFF;
        headingTime = currentTime;
      }
      else {
        if (relativeHeading < .25 && relativeHeading > -.25) {
          headingHold = 0;
          PID[HEADING].integratedError = 0;
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
          headingHold = updatePID(0, relativeHeading, &PID[HEADING]);
          headingTime = currentTime; // quick fix to soften heading hold, wait 100ms before applying heading hold
        }
      }
    }
    else {
      // minimum throttle not reached, use off settings
      setHeading = heading;
      headingHold = 0;
      PID[HEADING].integratedError = 0;
    }
  }
  // NEW SI Version
  commandedYaw = constrain(receiverData[YAW] * (2.5 * PWM2RAD) + radians(headingHold), -PI, PI);
  motorAxisCommand[YAW] = updatePID(commandedYaw, angle.value[YAW], &PID[YAW]);
  // uses flightAngle unbias rate
  //motors.setMotorAxisCommand(YAW, updatePID(commandedYaw, flightAngle->getGyroUnbias(YAW), &PID[YAW]));
}

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// processAltitudeHold //////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void processAltitudeHold(void)
{
  // ****************************** Altitude Adjust *************************
  // Thanks to Honk for his work with altitude hold
  // http://aeroquad.com/showthread.php?792-Problems-with-BMP085-I2C-barometer
  // Thanks to Sherbakov for his work in Z Axis dampening
  // http://aeroquad.com/showthread.php?359-Stable-flight-logic...&p=10325&viewfull=1#post10325
#ifdef AltitudeHold
  if (altitudeHold == ON) {
    throttleAdjust = updatePID(holdAltitude, altitude.getData(), &PID[ALTITUDE]);
    //throttleAdjust = constrain((holdAltitude - altitude.getData()) * PID[ALTITUDE].P, minThrottleAdjust, maxThrottleAdjust);
    throttleAdjust = constrain(throttleAdjust, minThrottleAdjust, maxThrottleAdjust);
    if (abs(holdThrottle - receiver.getData(THROTTLE)) > PANICSTICK_MOVEMENT) {
      altitudeHold = ALTPANIC; // too rapid of stick movement so PANIC out of ALTHOLD
    } else {
      if (receiver.getData(THROTTLE) > (holdThrottle + ALTBUMP)) { // AKA changed to use holdThrottle + ALTBUMP - (was MAXCHECK) above 1900
        holdAltitude += 0.01;
      }
      if (receiver.getData(THROTTLE) < (holdThrottle - ALTBUMP)) { // AKA change to use holdThorrle - ALTBUMP - (was MINCHECK) below 1100
        holdAltitude -= 0.01;
      }
    }
  }
  else {
    // Altitude hold is off, get throttle from receiver
    holdThrottle = receiver.getData(THROTTLE);
    throttleAdjust = autoDescent; // autoDescent is lowered from BatteryMonitor.h during battery alarm
  }
  // holdThrottle set in FlightCommand.pde if altitude hold is on
  throttle = holdThrottle + throttleAdjust; // holdThrottle is also adjust by BatteryMonitor.h during battery alarm
#else
  // If altitude hold not enabled in AeroQuad.pde, get throttle from receiver
  throttle = receiverData[THROTTLE] + autoDescent; //autoDescent is lowered from BatteryMonitor.h while battery critical, otherwise kept 0
#endif
}

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// processMinMaxMotorCommand ////////////////////////
//////////////////////////////////////////////////////////////////////////////
void processMinMaxMotorCommand(void)
{
  int maxMotor;
  
  for (byte motor = FIRSTMOTOR; motor < LASTMOTOR; motor++)
  {
    minCommand[motor] = MINTHROTTLE;
    maxCommand[motor] = MAXCOMMAND;
  }

  maxMotor = motorCommand[FIRSTMOTOR];
  
  for (byte motor=1; motor<LASTMOTOR; motor++)
    if (motorCommand[motor] > maxMotor) maxMotor = motorCommand[motor];
    
  for (byte motor = FIRSTMOTOR; motor < LASTMOTOR; motor++)
  {
    if (maxMotor > MAXCHECK)  
      motorCommand[motor] = motorCommand[motor] - (maxMotor - MAXCHECK);
  }
}

//////////////////////////////////////////////////////////////////////////////
///////////////////////////////// PLUS MODE //////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

#ifdef quadPlusConfig
void flightControl(void) {
  // ********************** Calculate Flight Error ***************************
  calculateFlightError();
  
  // ********************** Update Yaw ***************************************
  processHeading();

  // ********************** Altitude Adjust **********************************
  //processAltitudeHold();

  // ********************** Calculate Motor Commands *************************
  if (armed && safetyCheck) {
    motorCommand[FRONT] = (receiverData[THROTTLE] + autoDescent)                           - motorAxisCommand[PITCH] - motorAxisCommand[YAW];
    motorCommand[RIGHT] = (receiverData[THROTTLE] + autoDescent) - motorAxisCommand[ROLL]                            + motorAxisCommand[YAW];
    motorCommand[REAR]  = (receiverData[THROTTLE] + autoDescent)                           + motorAxisCommand[PITCH] - motorAxisCommand[YAW];
    motorCommand[LEFT]  = (receiverData[THROTTLE] + autoDescent) + motorAxisCommand[ROLL]                            + motorAxisCommand[YAW];
  } 

  // *********************** process min max motor command *******************
  processMinMaxMotorCommand();

  // Allows quad to do acrobatics by lowering power to opposite motors during hard manuevers
  if (flightMode == ACRO) {
    if (receiverData[ROLL] < MINCHECK) {        // Maximum Left Roll Rate
      minCommand[RIGHT] = MAXCOMMAND;
      maxCommand[LEFT]  = minAcro;
    }
    else if (receiverData[ROLL] > MAXCHECK) {   // Maximum Right Roll Rate
      minCommand[LEFT]  = MAXCOMMAND;
      maxCommand[RIGHT] = minAcro;
    }
    else if (receiverData[PITCH] < MINCHECK) {  // Maximum Nose Up Pitch Rate
      minCommand[FRONT] = MAXCOMMAND;
      maxCommand[REAR]  = minAcro;
    }
    else if (receiverData[PITCH] > MAXCHECK) {  // Maximum Nose Down Pitch Rate
      minCommand[REAR]  = MAXCOMMAND;
      maxCommand[FRONT] = minAcro;
    }
  }

  // Apply limits to motor commands
  for (byte motor = FIRSTMOTOR; motor < LASTMOTOR; motor++) {
    motorCommand[motor] = constrain(motorCommand[motor], minCommand[motor], maxCommand[motor]);
  }

  // If throttle in minimum position, don't apply yaw
  if (receiverData[THROTTLE] < MINCHECK) {
    for (byte motor = FIRSTMOTOR; motor < LASTMOTOR; motor++) {
      motorCommand[motor] = MINTHROTTLE;
    }
  }

  // ESC Calibration
  if (armed == OFF) {
    processCalibrateESC();
  }

  // *********************** Command Motors **********************
  if (armed == ON && safetyCheck == ON) {
    writeMotors(); // Defined in Motors.h
  }
}
#endif

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// X MODE //////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

#ifdef quadXConfig
void flightControl(void) {
  // ********************** Calculate Flight Error ***************************
  calculateFlightError();
  
  // ********************** Update Yaw ***************************************
  processHeading();

  // ********************** Altitude Adjust **********************************
  //processAltitudeHold();

  // ********************** Calculate Motor Commands *************************
  if (armed && safetyCheck) {
    motorCommand[FRONT_LEFT]  = (receiverData[THROTTLE] + autoDescent) + motorAxisCommand[ROLL] - motorAxisCommand[PITCH] - motorAxisCommand[YAW];
    motorCommand[FRONT_RIGHT] = (receiverData[THROTTLE] + autoDescent) - motorAxisCommand[ROLL] - motorAxisCommand[PITCH] + motorAxisCommand[YAW];
    motorCommand[REAR_RIGHT]  = (receiverData[THROTTLE] + autoDescent) - motorAxisCommand[ROLL] + motorAxisCommand[PITCH] - motorAxisCommand[YAW];
    motorCommand[REAR_LEFT]   = (receiverData[THROTTLE] + autoDescent) + motorAxisCommand[ROLL] + motorAxisCommand[PITCH] + motorAxisCommand[YAW];
  } 

  // *********************** process min max motor command *******************
  processMinMaxMotorCommand();

  // Allows quad to do acrobatics by lowering power to opposite motors during hard manuevers
  if (flightMode == ACRO) {
    if (receiverData[ROLL] < MINCHECK) {        // Maximum Left Roll Rate
      minCommand[FRONT_RIGHT] = MAXCOMMAND;
      minCommand[REAR_RIGHT]  =  MAXCOMMAND;
      maxCommand[FRONT_LEFT]  =  minAcro;
      maxCommand[REAR_LEFT]   =  minAcro;
    }
    else if (receiverData[ROLL] > MAXCHECK) {   // Maximum Right Roll Rate
      minCommand[FRONT_LEFT]  = MAXCOMMAND;
      minCommand[REAR_LEFT]   = MAXCOMMAND;
      maxCommand[FRONT_RIGHT] = minAcro;
      maxCommand[REAR_RIGHT]  = minAcro;
    }
    else if (receiverData[PITCH] < MINCHECK) {  // Maximum Nose Up Pitch Rate
      minCommand[FRONT_LEFT]  = MAXCOMMAND;
      minCommand[FRONT_RIGHT] = MAXCOMMAND;
      maxCommand[REAR_LEFT]   = minAcro;
      maxCommand[REAR_RIGHT]  = minAcro;
    }
    else if (receiverData[PITCH] > MAXCHECK) {  // Maximum Nose Down Pitch Rate
      minCommand[REAR_LEFT]   = MAXCOMMAND;
      minCommand[REAR_RIGHT]  = MAXCOMMAND;
      maxCommand[FRONT_LEFT]  = minAcro;
      maxCommand[FRONT_RIGHT] = minAcro;
    }
  }


  // Apply limits to motor commands
  for (byte motor = FIRSTMOTOR; motor < LASTMOTOR; motor++) {
    motorCommand[motor] = constrain(motorCommand[motor], minCommand[motor], maxCommand[motor]);
  }

  // If throttle in minimum position, don't apply yaw
  if (receiverData[THROTTLE] < MINCHECK) {
    for (byte motor = FIRSTMOTOR; motor < LASTMOTOR; motor++) {
      motorCommand[motor] = MINTHROTTLE;
    }
  }

  // ESC Calibration
  if (armed == OFF) {
    processCalibrateESC();
  }

  // *********************** Command Motors **********************
  if (armed == ON && safetyCheck == ON) {
    writeMotors(); // Defined in Motors.h
  }
}
#endif

//////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// Y4 MODE //////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

#ifdef y4Config
void flightControl(void) {
  // ********************** Calculate Flight Error ***************************
  calculateFlightError();
  
  // ********************** Update Yaw ***************************************
  processHeading();

  // ********************** Altitude Adjust **********************************
  //processAltitudeHold();

  // ********************** Calculate Motor Commands *************************
  if (armed && safetyCheck) {
    motorCommand[FRONT_LEFT]  =       (receiverData[THROTTLE] + autoDescent) + motorAxisCommand[ROLL] -       motorAxisCommand[PITCH] - motorAxisCommand[YAW];
    motorCommand[FRONT_RIGHT] =       (receiverData[THROTTLE] + autoDescent) - motorAxisCommand[ROLL] -       motorAxisCommand[PITCH] + motorAxisCommand[YAW];
    motorCommand[UPPER_REAR]  = 0.5 * (receiverData[THROTTLE] + autoDescent) - motorAxisCommand[ROLL] + 0.5 * motorAxisCommand[PITCH] - motorAxisCommand[YAW];
    motorCommand[LOWER_REAR]  = 0.5 * (receiverData[THROTTLE] + autoDescent) + motorAxisCommand[ROLL] + 0.5 * motorAxisCommand[PITCH] + motorAxisCommand[YAW];
  } 

  // *********************** process min max motor command *******************
  processMinMaxMotorCommand();

  // Allows quad to do acrobatics by lowering power to opposite motors during hard manuevers
  if (flightMode == ACRO) {
    if (receiverData[ROLL] < MINCHECK) {        // Maximum Left Roll Rate
      minCommand[FRONT_RIGHT] = MAXCOMMAND;
      maxCommand[FRONT_LEFT]  = minAcro;
    }
    else if (receiverData[ROLL] > MAXCHECK) {   // Maximum Right Roll Rate
      minCommand[FRONT_LEFT]  = MAXCOMMAND;
      maxCommand[FRONT_RIGHT] = minAcro;
    }
    else if (receiverData[PITCH] < MINCHECK) {  // Maximum Nose Up Pitch Rate
      minCommand[FRONT_LEFT]  = MAXCOMMAND;
      minCommand[FRONT_RIGHT] = MAXCOMMAND;
      maxCommand[UPPER_REAR]  = minAcro;
      maxCommand[LOWER_REAR]  = minAcro;
    }
    else if (receiverData[PITCH] > MAXCHECK) {  // Maximum Nose Down Pitch Rate
      minCommand[UPPER_REAR]  = MAXCOMMAND;
      minCommand[LOWER_REAR]  = MAXCOMMAND;
      maxCommand[FRONT_LEFT]  = minAcro;
      maxCommand[FRONT_RIGHT] = minAcro;
    }
  }

  // Apply limits to motor commands
  for (byte motor = FIRSTMOTOR; motor < LASTMOTOR; motor++) {
    motorCommand[motor] = constrain(motorCommand[motor], minCommand[motor], maxCommand[motor]);
  }

  // If throttle in minimum position, don't apply yaw
  if (receiverData[THROTTLE] < MINCHECK) {
    for (byte motor = FIRSTMOTOR; motor < LASTMOTOR; motor++) {
      motorCommand[motor] = MINTHROTTLE;
    }
  }

  // ESC Calibration
  if (armed == OFF) {
    processCalibrateESC();
  }

  // *********************** Command Motors **********************
  if (armed == ON && safetyCheck == ON) {
    writeMotors(); // Defined in Motors.h
  }
}
#endif

//////////////////////////////////////////////////////////////////////////////
///////////////////////////////// HEX PLUS MODE //////////////////////////////
//////////////////////////////////////////////////////////////////////////////

#ifdef hexPlusConfig
void flightControl(void) {
  // ********************** Calculate Flight Error ***************************
  calculateFlightError();
  
  // ********************** Update Yaw ***************************************
  processHeading();

  // ********************** Altitude Adjust **********************************
  //processAltitudeHold();

  // ********************** Calculate Motor Commands *************************
  if (armed && safetyCheck) {
    motorCommand[FRONT]       = (receiverData[THROTTLE] + autoDescent)                          - 0.866025 * motorAxisCommand[PITCH] - motorAxisCommand[YAW];
    motorCommand[FRONT_RIGHT] = (receiverData[THROTTLE] + autoDescent) - motorAxisCommand[ROLL] - 0.866025 * motorAxisCommand[PITCH] + motorAxisCommand[YAW];
    motorCommand[REAR_RIGHT]  = (receiverData[THROTTLE] + autoDescent) - motorAxisCommand[ROLL] + 0.866025 * motorAxisCommand[PITCH] - motorAxisCommand[YAW];
    motorCommand[REAR]        = (receiverData[THROTTLE] + autoDescent)                          + 0.866025 * motorAxisCommand[PITCH] + motorAxisCommand[YAW];
    motorCommand[REAR_LEFT]   = (receiverData[THROTTLE] + autoDescent) + motorAxisCommand[ROLL] + 0.866025 * motorAxisCommand[PITCH] - motorAxisCommand[YAW];
    motorCommand[FRONT_LEFT]  = (receiverData[THROTTLE] + autoDescent) + motorAxisCommand[ROLL] - 0.866025 * motorAxisCommand[PITCH] + motorAxisCommand[YAW];
  }
  
  // *********************** process min max motor command *******************
  processMinMaxMotorCommand();
  
  // Allows quad to do acrobatics by lowering power to opposite motors during hard manuevers

  if (flightMode == ACRO) {
    if (receiverData[ROLL] < MINCHECK) {        // Maximum Left Roll Rate
      minCommand[FRONT_RIGHT] = MAXCOMMAND;
      minCommand[REAR_RIGHT]  = MAXCOMMAND;
      maxCommand[FRONT_LEFT]  = minAcro;
      maxCommand[REAR_LEFT]   = minAcro;
    }
    else if (receiverData[ROLL] > MAXCHECK) {   // Maximum Right Roll Rate
      minCommand[FRONT_LEFT]  = MAXCOMMAND;
      minCommand[REAR_LEFT]   = MAXCOMMAND;
      maxCommand[FRONT_RIGHT] = minAcro;
      maxCommand[REAR_RIGHT]  = minAcro;
    }
    else if (receiverData[PITCH] < MINCHECK) {  // Maximum Nose Up Pitch Rate
      minCommand[FRONT]       = MAXCOMMAND;
      minCommand[FRONT_LEFT]  = MAXCOMMAND;
      minCommand[FRONT_RIGHT] = MAXCOMMAND;
      maxCommand[REAR]        = minAcro;
      maxCommand[REAR_LEFT]   = minAcro;
      maxCommand[REAR_RIGHT]  = minAcro;
    }
    else if (receiverData[PITCH] > MAXCHECK) {  // Maximum Nose Down Pitch Rate
      minCommand[REAR]        = MAXCOMMAND;
      minCommand[REAR_LEFT]   = MAXCOMMAND;
      minCommand[REAR_RIGHT]  = MAXCOMMAND;
      maxCommand[FRONT]       = minAcro;
      maxCommand[FRONT_LEFT]  = minAcro;
      maxCommand[FRONT_RIGHT] = minAcro;
    }
  }

  // Apply limits to motor commands
  for (byte motor = FIRSTMOTOR; motor < LASTMOTOR; motor++) {
    motorCommand[motor] = constrain(motorCommand[motor], minCommand[motor], maxCommand[motor]);
  }

  // If throttle in minimum position, don't apply yaw
  if (receiverData[THROTTLE] < MINCHECK) {
    for (byte motor = FIRSTMOTOR; motor < LASTMOTOR; motor++) {
      motorCommand[motor] = MINTHROTTLE;
    }
  }

  // ESC Calibration
  if (armed == OFF) {
    processCalibrateESC();
  }

  // *********************** Command Motors **********************
  if (armed == ON && safetyCheck == ON) {
    writeMotors(); // Defined in Motors.h
  }
} 
#endif

//////////////////////////////////////////////////////////////////////////////
///////////////////////////////// HEX X MODE /////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

#ifdef hexXConfig
void flightControl(void) {
  // ********************** Calculate Flight Error ***************************
  calculateFlightError();
  
  // ********************** Update Yaw ***************************************
  processHeading();

  // ********************** Altitude Adjust **********************************
  //processAltitudeHold();

  // ********************** Calculate Motor Commands *************************
  if (armed && safetyCheck) {
    motorCommand[FRONT_LEFT]  = (receiverData[THROTTLE] + autoDescent) + 0.866025 * motorAxisCommand[ROLL] - motorAxisCommand[PITCH] - motorAxisCommand[YAW];
    motorCommand[FRONT_RIGHT] = (receiverData[THROTTLE] + autoDescent) - 0.866025 * motorAxisCommand[ROLL] - motorAxisCommand[PITCH] + motorAxisCommand[YAW];
    motorCommand[RIGHT]       = (receiverData[THROTTLE] + autoDescent) - 0.866025 * motorAxisCommand[ROLL]                           - motorAxisCommand[YAW];
    motorCommand[REAR_RIGHT]  = (receiverData[THROTTLE] + autoDescent) - 0.866025 * motorAxisCommand[ROLL] + motorAxisCommand[PITCH] + motorAxisCommand[YAW];
    motorCommand[REAR_LEFT]   = (receiverData[THROTTLE] + autoDescent) + 0.866025 * motorAxisCommand[ROLL] + motorAxisCommand[PITCH] - motorAxisCommand[YAW];
    motorCommand[LEFT]        = (receiverData[THROTTLE] + autoDescent) + 0.866025 * motorAxisCommand[ROLL]                           + motorAxisCommand[YAW];
  }
  
  // *********************** process min max motor command *******************
  processMinMaxMotorCommand();

  // Allows quad to do acrobatics by lowering power to opposite motors during hard manuevers

  if (flightMode == ACRO) {
    if (receiverData[ROLL] < MINCHECK) {        // Maximum Left Roll Rate
      minCommand[RIGHT]       = MAXCOMMAND;
      minCommand[FRONT_RIGHT] = MAXCOMMAND;
      minCommand[REAR_RIGHT]  = MAXCOMMAND;
      maxCommand[LEFT]        = minAcro;
      maxCommand[FRONT_LEFT]  = minAcro;
      maxCommand[REAR_LEFT]   = minAcro;
    }
    else if (receiverData[ROLL] > MAXCHECK) {   // Maximum Right Roll Rate
      minCommand[LEFT]        = MAXCOMMAND;
      minCommand[FRONT_LEFT]  = MAXCOMMAND;
      minCommand[REAR_LEFT]   = MAXCOMMAND;
      maxCommand[RIGHT]       = minAcro;
      maxCommand[FRONT_RIGHT] = minAcro;
      maxCommand[REAR_RIGHT]  = minAcro;
    }
    else if (receiverData[PITCH] < MINCHECK) {  // Maximum Nose Up Pitch Rate
      minCommand[FRONT_LEFT]  = MAXCOMMAND;
      minCommand[FRONT_RIGHT] = MAXCOMMAND;
      maxCommand[REAR_LEFT]   = minAcro;
      maxCommand[REAR_RIGHT]  = minAcro;
    }
    else if (receiverData[PITCH] > MAXCHECK) {  // Maximum Nose Down Pitch Rate
      minCommand[REAR_LEFT]   = MAXCOMMAND;
      minCommand[REAR_RIGHT]  = MAXCOMMAND;
      maxCommand[FRONT_LEFT]  = minAcro;
      maxCommand[FRONT_RIGHT] = minAcro;
    }
  }

  // Apply limits to motor commands
  for (byte motor = FIRSTMOTOR; motor < LASTMOTOR; motor++) {
    motorCommand[motor]= constrain(motorCommand[motor], minCommand[motor], maxCommand[motor]);
  }

  // If throttle in minimum position, don't apply yaw
  if (receiverData[THROTTLE] < MINCHECK) {
    for (byte motor = FIRSTMOTOR; motor < LASTMOTOR; motor++) {
      motorCommand[motor] = MINTHROTTLE;
    }
  }

  // ESC Calibration
  if (armed == OFF) {
    processCalibrateESC();
  }

  // *********************** Command Motors **********************
  if (armed == ON && safetyCheck == ON) {
    writeMotors(); // Defined in Motors.h
  }
} 
#endif

//////////////////////////////////////////////////////////////////////////////
////////////////////////////////// Y6 MODE ///////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

#ifdef y6Config
void flightControl(void) {
  // ********************** Calculate Flight Error ***************************
  calculateFlightError();
  
  // ********************** Update Yaw ***************************************
  processHeading();

  // ********************** Altitude Adjust **********************************
  //processAltitudeHold();

  // ********************** Calculate Motor Commands *************************
  if (armed && safetyCheck) {
    motorCommand[UPPER_FRONT_LEFT]  = (receiverData[THROTTLE] + autoDescent) + motorAxisCommand[ROLL] - 0.866025 * motorAxisCommand[PITCH] - motorAxisCommand[YAW];
    motorCommand[UPPER_FRONT_RIGHT] = (receiverData[THROTTLE] + autoDescent) - motorAxisCommand[ROLL] - 0.866025 * motorAxisCommand[PITCH] - motorAxisCommand[YAW];
    motorCommand[UPPER_REAR]        = (receiverData[THROTTLE] + autoDescent)                          + 0.866025 * motorAxisCommand[PITCH] + motorAxisCommand[YAW];
    motorCommand[LOWER_FRONT_LEFT]  = (receiverData[THROTTLE] + autoDescent) + motorAxisCommand[ROLL] + 0.866025 * motorAxisCommand[PITCH] + motorAxisCommand[YAW];
    motorCommand[LOWER_FRONT_RIGHT] = (receiverData[THROTTLE] + autoDescent) - motorAxisCommand[ROLL] + 0.866025 * motorAxisCommand[PITCH] + motorAxisCommand[YAW];
    motorCommand[LOWER_REAR]        = (receiverData[THROTTLE] + autoDescent)                          - 0.866025 * motorAxisCommand[PITCH] - motorAxisCommand[YAW];
  }
  
  // *********************** process min max motor command *******************
  processMinMaxMotorCommand();

  // Allows quad to do acrobatics by lowering power to opposite motors during hard manuevers

  if (flightMode == ACRO) {
    if (receiverData[ROLL] < MINCHECK) {        // Maximum Left Roll Rate
      minCommand[UPPER_FRONT_RIGHT] = MAXCOMMAND;
      minCommand[LOWER_FRONT_RIGHT] = MAXCOMMAND;
      maxCommand[UPPER_FRONT_LEFT]  = minAcro;
      maxCommand[LOWER_FRONT_LEFT]  = minAcro;
    }
    else if (receiverData[ROLL] > MAXCHECK) {   // Maximum Right Roll Rate
      minCommand[UPPER_FRONT_LEFT]  = MAXCOMMAND;
      minCommand[LOWER_FRONT_LEFT]  = MAXCOMMAND;
      maxCommand[UPPER_FRONT_RIGHT] = minAcro;
      maxCommand[LOWER_FRONT_RIGHT] = minAcro;
    }
    else if (receiverData[PITCH] < MINCHECK) {  // Maximum Nose Up Pitch Rate
      minCommand[UPPER_FRONT_LEFT]  = MAXCOMMAND;
      minCommand[UPPER_FRONT_RIGHT] = MAXCOMMAND;
      minCommand[LOWER_FRONT_LEFT]  = MAXCOMMAND;
      maxCommand[LOWER_FRONT_RIGHT] = MAXCOMMAND;
      maxCommand[UPPER_REAR]        = minAcro;
      maxCommand[LOWER_REAR]        = minAcro;
    }
    else if (receiverData[PITCH] > MAXCHECK) {  // Maximum Nose Down Pitch Rate
      minCommand[UPPER_REAR]        = MAXCOMMAND;
      minCommand[LOWER_REAR]        = MAXCOMMAND;
      minCommand[UPPER_FRONT_LEFT]  = minAcro;
      maxCommand[UPPER_FRONT_RIGHT] = minAcro;
      maxCommand[LOWER_FRONT_LEFT]  = minAcro;
      maxCommand[LOWER_FRONT_RIGHT] = minAcro;
    }
  }

  // Apply limits to motor commands
  for (byte motor = FIRSTMOTOR; motor < LASTMOTOR; motor++) {
    motorCommand[motor] = constrain(motorCommand[motor], minCommand[motor], maxCommand[motor]);
  }

  // If throttle in minimum position, don't apply yaw
  if (receiverData[THROTTLE] < MINCHECK) {
    for (byte motor = FIRSTMOTOR; motor < LASTMOTOR; motor++) {
      motorCommand[motor] =  MINTHROTTLE;
    }
  }

  // ESC Calibration
  if (armed == OFF) {
    processCalibrateESC();
  }

  // *********************** Command Motors **********************
  if (armed == ON && safetyCheck == ON) {
    writeMotors(); // Defined in Motors.h
  }
} 
#endif
