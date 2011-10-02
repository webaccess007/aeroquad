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

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// calculateFlightError /////////////////////////////
//////////////////////////////////////////////////////////////////////////////

#define RATE_SCALING     0.01   // Stick to rate scaling (5 radians/sec)/(500 RX PWMs) = 0.01
#define ATTITUDE_SCALING 0.002  // Stick to att scaling (1 radian)/(500 RX PWMs) = 0.002

//////////////////////////////////////////////////////////////////////////////

float attCmd[3];
float attPID[3];

float rateCmd[3];

float headingReference;

unsigned int fcCurrentTime;
unsigned int fcPreviousTime;
float fcDt;
boolean fcFirstPass = 1;

//////////////////////////////////////////////////////////////////////////////

void calculateFlightCommands(void)
{
  if (fcFirstPass == 1)
  {
    fcCurrentTime  = micros();
    fcPreviousTime = fcCurrentTime;
    fcFirstPass    = 0;
    headingReference = angle.value[YAW];
  }
  
  fcCurrentTime = micros();
  fcDt = float(fcCurrentTime-fcPreviousTime)/1000000;
  fcPreviousTime = fcCurrentTime;

  if (flightMode == ATTITUDE)
  {
    attCmd[ROLL]  = receiverData[ROLL]  * ATTITUDE_SCALING;
    attCmd[PITCH] = receiverData[PITCH] * ATTITUDE_SCALING;
  }
  //else
  //{
  //  attCmd[ROLL]  = velocityPID[LATERAL];
  //  attCmd[PITCH] = velocityPID[LONGITUDINAL];
  //}
  
  if (flightMode >= ATTITUDE)
  {
    attPID[ROLL]  = updatePID(attCmd[ROLL],   angle.value[ROLL],  fcDt, &PID[ROLL_ATT_PID], 1);
    attPID[PITCH] = updatePID(attCmd[PITCH], -angle.value[PITCH], fcDt, &PID[PITCH_ATT_PID], 1);
  }
  
  if (flightMode == RATE)
  {
    rateCmd[ROLL]  = receiverData[ROLL]  * RATE_SCALING;
    rateCmd[PITCH] = receiverData[PITCH] * RATE_SCALING;
  }
  else
  {
    rateCmd[ROLL]  = attPID[ROLL];
    rateCmd[PITCH] = attPID[PITCH];
  }
  
  if ((commandInDetent[YAW] == TRUE) && (headingHoldAvailable == TRUE))  // Heading Hold is ON
  {
    if (previousCommandInDetent[YAW] == FALSE)
    {
      setIntegralError(HEADING_PID, 0.0);  // First pass in heading hold with new reference, zero integral PID error
    }
    rateCmd[YAW] = updatePID(headingReference, angle.value[YAW], fcDt, &PID[HEADING_PID], 1);
  }
  else  // Heading Hold is OFF
  {
    rateCmd[YAW] = receiverData[YAW] * RATE_SCALING;
    headingReference = angle.value[YAW];
  }
  
  previousCommandInDetent[YAW] = commandInDetent[YAW];
  
  motorAxisCommand[ROLL]  = updatePID(rateCmd[ROLL],   gyro.value[ROLL],  fcDt, &PID[ROLL_RATE_PID], 0);
  motorAxisCommand[PITCH] = updatePID(rateCmd[PITCH], -gyro.value[PITCH], fcDt, &PID[PITCH_RATE_PID], 0);
  motorAxisCommand[YAW]   = updatePID(rateCmd[YAW],    gyro.value[YAW],   fcDt, &PID[YAW_RATE_PID], 0);
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
//////////////////////////////// FLIGHT CONTROL //////////////////////////////
//////////////////////////////////////////////////////////////////////////////

void flightControl(void) {
  // ********************** Calculate Flight Commands ************************
  calculateFlightCommands();
  
  // ********************** Calculate Motor Commands *************************
  if (armed && safetyCheck) {
    calculateMotorCommands();
  } 

  // ********************** Process Min/Max Motor Command ********************
  processMinMaxMotorCommand();

  // ********************** Process Hard Maneuvers ***************************
  processHardManeuvers();
  
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

  if (armed == ON && safetyCheck == ON)
    writeMotors();
    
  if (armed == OFF && escsCalibrating == OFF)
    commandAllMotors(MINCOMMAND);
}

//////////////////////////////////////////////////////////////////////////////
