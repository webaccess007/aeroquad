/*
  AeroQuad v1.3.2 - September 2009
 www.AeroQuad.info
 Copyright (c) 2009 Ted Carancho.  All rights reserved.
 An Open Source Arduino based quadrocopter.
 
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

void readSerialCommand(HardwareSerial *serialPort, char *queryType) 
{
  // Check for serial message
  if (serialPort->available()) 
  {
    digitalWrite(LEDPIN, LOW);
    *queryType = serialPort->read(); 

    switch (*queryType)
    {

    case 'A': // Receive roll and pitch gyro PID
      PID[ROLL].P = readFloatSerial(serialPort);
      PID[ROLL].I = readFloatSerial(serialPort);
      PID[ROLL].D = readFloatSerial(serialPort);
      PID[ROLL].lastPosition = 0;
      PID[ROLL].integratedError = 0;
      PID[PITCH].P = readFloatSerial(serialPort);
      PID[PITCH].I = readFloatSerial(serialPort);
      PID[PITCH].D = readFloatSerial(serialPort);
      PID[PITCH].lastPosition = 0;
      PID[PITCH].integratedError = 0;
      break;

    case 'C': // Receive yaw PID
      PID[YAW].P = readFloatSerial(serialPort);
      PID[YAW].I = readFloatSerial(serialPort);
      PID[YAW].D = readFloatSerial(serialPort);
      PID[YAW].lastPosition = 0;
      PID[YAW].integratedError = 0;
      break;
    case 'E': // Receive roll and pitch auto level PID
      PID[LEVELROLL].P = readFloatSerial(serialPort);
      PID[LEVELROLL].I = readFloatSerial(serialPort);
      PID[LEVELROLL].D = readFloatSerial(serialPort);
      PID[LEVELROLL].lastPosition = 0;
      PID[LEVELROLL].integratedError = 0;
      PID[LEVELPITCH].P = readFloatSerial(serialPort);
      PID[LEVELPITCH].I = readFloatSerial(serialPort);
      PID[LEVELPITCH].D = readFloatSerial(serialPort);
      PID[LEVELPITCH].lastPosition = 0;
      PID[LEVELPITCH].integratedError = 0;
      break;
    case 'G': // Receive auto level configuration
      levelLimit = readFloatSerial(serialPort);
      levelOff = readFloatSerial(serialPort);
      break;
    case 'I': // Receive flight control configuration
      windupGuard = readFloatSerial(serialPort);
      xmitFactor = readFloatSerial(serialPort);
      break;
    case 'K': // Receive data filtering values
      smoothFactor[GYRO] = readFloatSerial(serialPort);
      smoothFactor[ACCEL] = readFloatSerial(serialPort);
      timeConstant = readFloatSerial(serialPort);
      break;
    case 'M': // Receive motor smoothing values
      smoothTransmitter[ROLL] = readFloatSerial(serialPort);
      smoothTransmitter[PITCH] = readFloatSerial(serialPort);
      smoothTransmitter[YAW] = readFloatSerial(serialPort);
      smoothTransmitter[THROTTLE] = readFloatSerial(serialPort);
      smoothTransmitter[MODE] = readFloatSerial(serialPort);
      smoothTransmitter[AUX] = readFloatSerial(serialPort);
      break;
    case 'O': // Received transmitter calibrtion values
      mTransmitter[ROLL] = readFloatSerial(serialPort);
      bTransmitter[ROLL] = readFloatSerial(serialPort);
      mTransmitter[PITCH] = readFloatSerial(serialPort);
      bTransmitter[PITCH] = readFloatSerial(serialPort);
      mTransmitter[YAW] = readFloatSerial(serialPort);
      bTransmitter[YAW] = readFloatSerial(serialPort);
      mTransmitter[THROTTLE] = readFloatSerial(serialPort);
      bTransmitter[THROTTLE] = readFloatSerial(serialPort);
      mTransmitter[MODE] = readFloatSerial(serialPort);
      bTransmitter[MODE] = readFloatSerial(serialPort);
      mTransmitter[AUX] = readFloatSerial(serialPort);
      bTransmitter[AUX] = readFloatSerial(serialPort);
      break;
    case 'W': // Write all user configurable values to EEPROM
      writeFloat(PID[ROLL].P, PGAIN_ADR);
      writeFloat(PID[ROLL].I, IGAIN_ADR);
      writeFloat(PID[ROLL].D, DGAIN_ADR);
      writeFloat(PID[PITCH].P, PITCH_PGAIN_ADR);
      writeFloat(PID[PITCH].I, PITCH_IGAIN_ADR);
      writeFloat(PID[PITCH].D, PITCH_DGAIN_ADR);
      writeFloat(PID[LEVELROLL].P, LEVEL_PGAIN_ADR);
      writeFloat(PID[LEVELROLL].I, LEVEL_IGAIN_ADR);
      writeFloat(PID[LEVELROLL].D, LEVEL_DGAIN_ADR);
      writeFloat(PID[LEVELPITCH].P, LEVEL_PITCH_PGAIN_ADR);
      writeFloat(PID[LEVELPITCH].I, LEVEL_PITCH_IGAIN_ADR);
      writeFloat(PID[LEVELPITCH].D, LEVEL_PITCH_DGAIN_ADR);
      writeFloat(PID[YAW].P, YAW_PGAIN_ADR);
      writeFloat(PID[YAW].I, YAW_IGAIN_ADR);
      writeFloat(PID[YAW].D, YAW_DGAIN_ADR);
      writeFloat(windupGuard, WINDUPGUARD_ADR);  
      writeFloat(levelLimit, LEVELLIMIT_ADR);   
      writeFloat(levelOff, LEVELOFF_ADR); 
      writeFloat(xmitFactor, XMITFACTOR_ADR);
      writeFloat(smoothFactor[GYRO], GYROSMOOTH_ADR);
      writeFloat(smoothFactor[ACCEL], ACCSMOOTH_ADR);
      writeFloat(smoothTransmitter[THROTTLE], THROTTLESMOOTH_ADR);
      writeFloat(smoothTransmitter[ROLL], ROLLSMOOTH_ADR);
      writeFloat(smoothTransmitter[PITCH], PITCHSMOOTH_ADR);
      writeFloat(smoothTransmitter[YAW], YAWSMOOTH_ADR);
      writeFloat(smoothTransmitter[MODE], MODESMOOTH_ADR);
      writeFloat(smoothTransmitter[AUX], AUXSMOOTH_ADR);
      writeFloat(timeConstant, FILTERTERM_ADR);
      writeFloat(mTransmitter[THROTTLE], THROTTLESCALE_ADR);
      writeFloat(bTransmitter[THROTTLE], THROTTLEOFFSET_ADR);
      writeFloat(mTransmitter[ROLL], ROLLSCALE_ADR);
      writeFloat(bTransmitter[ROLL], ROLLOFFSET_ADR);
      writeFloat(mTransmitter[PITCH], PITCHSCALE_ADR);
      writeFloat(bTransmitter[PITCH], PITCHOFFSET_ADR);
      writeFloat(mTransmitter[YAW], YAWSCALE_ADR);
      writeFloat(bTransmitter[YAW], YAWOFFSET_ADR);
      writeFloat(mTransmitter[MODE], MODESCALE_ADR);
      writeFloat(bTransmitter[MODE], MODEOFFSET_ADR);
      writeFloat(mTransmitter[AUX], AUXSCALE_ADR);
      writeFloat(bTransmitter[AUX], AUXOFFSET_ADR);
      zeroIntegralError();
      // Complementary filter setup
      configureFilter(timeConstant);
      break;
    case 'Y': // Initialize EEPROM with default values
      PID[ROLL].P = 3.75;
      PID[ROLL].I = 0;
      PID[ROLL].D = -10;
      PID[PITCH].P = 3.75;
      PID[PITCH].I = 0;
      PID[PITCH].D = -10;
      PID[YAW].P = 12.0;
      PID[YAW].I = 0;
      PID[YAW].D = 0;
      PID[LEVELROLL].P = 2;
      PID[LEVELROLL].I = 0;
      PID[LEVELROLL].D = 0;
      PID[LEVELPITCH].P = 2;
      PID[LEVELPITCH].I = 0;
      PID[LEVELPITCH].D = 0;
      windupGuard = 2000.0;
      xmitFactor = 0.20;  
      levelLimit = 2000.0;
      levelOff = 50;  
      smoothFactor[GYRO] = 0.20;
      smoothFactor[ACCEL] = 0.20;
      timeConstant = 3.0;   
      for (channel = ROLL; channel < LASTCHANNEL; channel++) {
        mTransmitter[channel] = 1.0;
        bTransmitter[channel] = 0.0;
      }
      smoothTransmitter[THROTTLE] = 1.0;
      smoothTransmitter[ROLL] = 1.0;
      smoothTransmitter[PITCH] = 1.0;
      smoothTransmitter[YAW] = 0.5;  
      smoothTransmitter[MODE] = 1.0;
      smoothTransmitter[AUX] = 1.0;
      smoothHeading = 1.0;

      zeroGyros();
      zeroAccelerometers();
      zeroIntegralError();
      break;
    case '1': // Calibrate ESCS's by setting Throttle high on all channels
      armed = 0;
      calibrateESC = 1;
      break;
    case '2': // Calibrate ESC's by setting Throttle low on all channels
      armed = 0;
      calibrateESC = 2;
      break;
    case '3': // Test ESC calibration
      armed = 0;
      testCommand = readFloatSerial(serialPort);
      calibrateESC = 3;
      break;
    case '4': // Turn off ESC calibration
      armed = 0;
      calibrateESC = 0;
      testCommand = 1000;
      break;        
    case '5': // Send individual motor commands (motor, command)
      armed = 0;
      calibrateESC = 5;
      for (motor = FRONT; motor < LASTMOTOR; motor++)
        remoteCommand[motor] = readFloatSerial(serialPort);
      break;
    case 'a': // Enable/disable fast data transfer of sensor data
      *queryType = 'X'; // Stop any other telemetry streaming
      if (readFloatSerial(serialPort) == 1)
        fastTransfer = ON;
      else
        fastTransfer = OFF;
      break;
    case 'b': // calibrate gyros
      zeroGyros();
      break;
    case 'c': // calibrate accels
      zeroAccelerometers();
      break;
    case 'd': // update Heading smoothing
      smoothHeading = readFloatSerial(serialPort);
    }
    digitalWrite(LEDPIN, HIGH);
  }
}

// Used to read floating point values from the serial port
float readFloatSerial(HardwareSerial *serialPort) 
{
  byte index = 0;
  byte timeout = 0;
  char data[128] = "";

  do {
    if (serialPort->available() == 0) {
      delay(10);
      timeout++;
    }
    else {
      data[index] = serialPort->read();
      timeout = 0;
      index++;
    }
  }  
  while ((data[limitRange(index-1, 0, 128)] != ';') && (timeout < 5) && (index < 128));
  return atof(data);
}