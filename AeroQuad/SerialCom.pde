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

// SerialCom.pde is responsible for the serial communication for commands and telemetry from the AeroQuad
// This comtains readSerialCommand() which listens for a serial command and it's arguments
// This also contains readSerialTelemetry() which listens for a telemetry request and responds with the requested data
// For more information on each command/telemetry look at: http://aeroquad.com/content.php?117

// Includes re-write / fixes from Aadamson and ala42, special thanks to those guys!
// http://aeroquad.com/showthread.php?1461-We-have-some-hidden-warnings&p=14618&viewfull=1#post14618

//***************************************************************************************************
//********************************** Serial Commands ************************************************
//***************************************************************************************************
#if !defined(I2C_ESC)
  bool validateCalibrateCommand()
  {
    if ((readFloatSerial() == 123.45) & (armed == OFF))  // use a specific float value to validate full throttle call is being sent
      return true;
    else
      return false;
  }
#endif

void readSerialPID(unsigned char PIDid) {
  struct PIDdata* pid = &PID[PIDid];
  pid->P = readFloatSerial();
  pid->I = readFloatSerial();
  pid->D = readFloatSerial();
  pid->windupGuard = readFloatSerial();
  pid->lastState = 0;
  pid->iTerm = 0;
  pid->firstPass = true;
}

void readSerialCommand() {
  // Check for serial message
  if (SERIAL_AVAILABLE()) {
    digitalWrite(LEDPIN, LOW);
    queryType = SERIAL_READ();
    switch (queryType) {
    
    case 'A':  // Receive Roll Rate PID values
      readSerialPID(ROLL_RATE_PID);
      break;
    
    case 'B':  // Receive Pitch Rate PID values
      readSerialPID(PITCH_RATE_PID);
      break;
      
    case 'C':  // Receive Yaw Rate PID values
      readSerialPID(YAW_RATE_PID);
      break;
      
    case 'D':  // Receive minAcro value
      minAcro = readFloatSerial();
      break;
      
    case 'E':  // Receive Roll Attitude PID values
      readSerialPID(ROLL_ATT_PID);
      break;
      
    case 'F':  // Receive Pitch Attitude PID values
      readSerialPID(PITCH_ATT_PID);
      break;
      
    case 'G':  // Receive Heading PID values
      readSerialPID(HEADING_PID);
      break;

    case 'I': // Initialize EEPROM with default values
      initializeEEPROM();
      computeGyroBias();
      computeAccelBias();
      zeroIntegralError();
      break;
      
    case 'K': // calibrate gyros
      computeGyroBias();
      break;
      
    case 'L': // calibrate accels
      computeAccelBias();
      break;     

    case 'W': // Write all user configurable values to EEPROM
      writeEEPROM(); // defined in DataStorage.h
      zeroIntegralError();
      break;
      
    #if !defined(I2C_ESC)
      case '1': // Calibrate ESCS's by setting MAXCOMMAND on all motors
      	if (validateCalibrateCommand)
          commandAllMotors(MAXCOMMAND);
        break;
        
      case '2': // Calibrate ESC's by setting MINICOMMAND on all motors
      	if (validateCalibrateCommand);
          commandAllMotors(MINCOMMAND);
        break;
        
      case '3': // Test ESC calibration
        if (validateCalibrateCommand)
          commandAllMotors(constrain(readFloatSerial(), MINCOMMAND, MAXCOMMAND));
        break;
    #endif
        
    case '5': // Send individual motor commands (motor, command)
      if (validateCalibrateCommand) {
        for (byte motor = FIRSTMOTOR; motor < LASTMOTOR; motor++)
        {
          remoteMotorCommand[motor] = readFloatSerial();
          motorCommand[motor] = constrain(remoteMotorCommand[motor], MINCOMMAND, MAXCOMMAND);
        }
        writeMotors();
      }
      break;
    }
    digitalWrite(LEDPIN, HIGH);
  }
}

//***************************************************************************************************
//********************************* Serial Telemetry ************************************************
//***************************************************************************************************

void PrintValueComma(float val) {
  SERIAL_PRINT(val);
  comma();
}

void PrintValueComma(double val) {
  SERIAL_PRINT(val);
  comma();
}

void PrintValueComma(char val) {
  SERIAL_PRINT(val);
  comma();
}

void PrintValueComma(int val) {
  SERIAL_PRINT(val);
  comma();
}

void PrintValueComma(unsigned long val)
{
  SERIAL_PRINT(val);
  comma();
}

void printPID(unsigned char IDPid)
{
  PrintValueComma(PID[IDPid].P);
  PrintValueComma(PID[IDPid].I);
  PrintValueComma(PID[IDPid].D);
  SERIAL_PRINTLN(PID[IDPid].windupGuard);
}

void sendSerialTelemetry() {
  update = 0;
  switch (queryType) {
  
  case 'a':  // Send Roll Rate PID values
    printPID(ROLL_RATE_PID);
    queryType = 'x';
    break;
    
  case 'b':  // Send Pitch Rate PID values
    printPID(PITCH_RATE_PID);
    queryType = 'x';
    break;
    
  case 'c':  // Send Yaw Rate PID values
    printPID(YAW_RATE_PID);
    queryType = 'x';
    break;
    
  case 'd':  // Send minAcro
    SERIAL_PRINTLN(minAcro);
    queryType = 'x';
    break;
    
  case 'e':  // Send Roll Attiude PID values
    printPID(ROLL_ATT_PID);
    queryType = 'x';
    break;
    
  case 'f':  // Send Pitch Attitude PID values
    printPID(PITCH_ATT_PID);
    queryType = 'x';
    break;
    
  case 'g':  // Send Heading PID values
    printPID(HEADING_PID);
    queryType = 'x';
    break;
    
  case 'h': // Send Filtered Accels
    PrintValueComma(filteredAccel.value[XAXIS]);
    PrintValueComma(filteredAccel.value[YAXIS]);
    SERIAL_PRINTLN(filteredAccel.value[ZAXIS]);
    break;
    
  case 'i': // Send Rate Gyros
    PrintValueComma(gyro.value[ROLL]  * 57.3);
    PrintValueComma(gyro.value[PITCH] * 57.3);
    SERIAL_PRINTLN(gyro.value[YAW]    * 57.3);
    break;
    
  case 'j': // Send Attitudes
    PrintValueComma(angle.value[ROLL]  * 57.3);
    PrintValueComma(angle.value[PITCH] * 57.3);
    SERIAL_PRINTLN(angle.value[YAW]    * 57.3);
    break;
    
  #if defined(HMC5843) | defined(HMC5883)
  case 'k': // Send Raw Mag
    PrintValueComma(mag.value[XAXIS]);
    PrintValueComma(mag.value[YAXIS]);
    SERIAL_PRINTLN(mag.value[ZAXIS]);
    break;
  #endif
  
  case 'm': // Send Motor Commands 1 thru 4
    #if (LASTMOTOR == 4)
      PrintValueComma(motorCommand[0]);
      PrintValueComma(motorCommand[1]);
      PrintValueComma(motorCommand[2]);
      SERIAL_PRINTLN(motorCommand[3]);
    #else
      PrintValueComma(motorCommand[0]);
      PrintValueComma(motorCommand[1]);
      PrintValueComma(motorCommand[2]);
      PrintValueComma(motorCommand[3]);
      PrintValueComma(motorCommand[4]);
      SERIAL_PRINTLN(motorCommand[5]);
    #endif
    break;
  
  case 'n': // Send Motor Axis Commands
    PrintValueComma(motorAxisCommand[ROLL]);
    PrintValueComma(motorAxisCommand[PITCH]);
    SERIAL_PRINTLN(motorAxisCommand[YAW]);
    break;
    
  case 'r': // Send Receiver Commands 1 thru 5
    PrintValueComma(receiverData[ROLL]);
    PrintValueComma(receiverData[PITCH]);
    PrintValueComma(receiverData[YAW]);
    PrintValueComma(receiverData[THROTTLE]);
    SERIAL_PRINTLN(receiverData[MODE]);
    
  case 'x': // Stop sending messages
    break;

  case 'z': // Send flight software version
    SERIAL_PRINTLN(VERSION, 1);
    #if defined(AeroQuad_v18)
      SERIAL_PRINTLN("V18");
    #elif defined(AeroQuadMega_v2)
      SERIAL_PRINTLN("V2");
    #elif defined(AeroQuad_Mini) | defined(AeroQuad_Mini_FFIMUV2)
      SERIAL_PRINTLN("Mini");
    #endif
    #if defined(quadPlusConfig)
      SERIAL_PRINTLN("Quad +");
    #elif defined(quadXConfig)
      SERIAL_PRINTLN("Quad X");
    #elif defined(y4Config)
      SERIAL_PRINTLN("Y4");
    #elif defined(hexPlusConfig)
      SERIAL_PRINTLN("Hex +");
    #elif defined(hexXConfig)
      SERIAL_PRINTLN("Hex X");
    #elif defined(y6Config)
      SERIAL_PRINTLN("Y6");
    #endif
    queryType = 'X';
    break;
    
  #if !defined(I2C_ESC)
    case '6': // Report remote commands
      #if (LASTMOTOR == 4)
        PrintValueComma(remoteMotorCommand[0]);
        PrintValueComma(remoteMotorCommand[1]);
        PrintValueComma(remoteMotorCommand[2]);
        SERIAL_PRINTLN(remoteMotorCommand[3]);
      #else
        PrintValueComma(remoteMotorCommand[0]);
        PrintValueComma(remoteMotorCommand[1]);
        PrintValueComma(remoteMotorCommand[2]);
        PrintValueComma(remoteMotorCommand[3]);
        PrintValueComma(remoteMotorCommand[4]);
        SERIAL_PRINTLN(remoteMotorCommand[5]);
      #endif
      queryType = 'X';
      break;
  #endif
    
  case '=': // Send Free Form Debug
    // What are you looking at?  And why?
    break;
  }
}

// Used to read floating point values from the serial port
float readFloatSerial() {
  #define SERIALFLOATSIZE 10
  byte index = 0;
  byte timeout = 0;
  char data[SERIALFLOATSIZE] = "";

  do {
    if (SERIAL_AVAILABLE() == 0) {
      delay(10);
      timeout++;
    }
    else {
      data[index] = SERIAL_READ();
      timeout = 0;
      index++;
    }
  } while ((index == 0 || data[index-1] != ';') && (timeout < 5) && (index < sizeof(data)-1));
  data[index] = '\0';

  return atof(data);
}

void comma() {
  SERIAL_PRINT(',');
}




