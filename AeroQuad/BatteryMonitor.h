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


// *******************************************************************************
// ************************ AeroQuad Battery Monitor *****************************
// *******************************************************************************

#define BATTERYPIN 0
#define OK         0
#define WARNING    1
#define ALARM      2

byte  batteryStatus;
float lowVoltageWarning;  // Pack voltage at which to trigger alarm (first alarm)
float lowVoltageAlarm;    // Pack voltage at which to trigger alarm (critical alarm)
float batteryVoltage;

byte state, firstAlarm;
float diode; // raw voltage goes through diode on Arduino
float batteryScaleFactor;
long  currentBatteryTime, previousBatteryTime;

void initializeBatteryMonitor(void)
{
  float R1   = 15000;
  float R2   =  7500;
  float Aref =     5.0;
  batteryScaleFactor = ((Aref / 1024.0) * ((R1 + R2) / R2));
  #ifdef AeroQuad_Mini
    diode = 0.53; // measured with DMM
  #else    
    diode = 0.9; // measured with DMM
  #endif    
  analogReference(DEFAULT);
  previousBatteryTime = millis();
  state = LOW;
  firstAlarm = OFF;
  
  lowVoltageWarning = 10.2; //10.8;
  lowVoltageAlarm   = 9.5; //10.2;
  batteryVoltage    = lowVoltageWarning + 2;
  batteryStatus     = OK;
}

void lowBatteryEvent(byte level) {
  long currentBatteryTime = millis() - previousBatteryTime;
  if (level == OK) {
    autoDescent = 0;
  }
  if (level == WARNING) {
    if (currentBatteryTime > 1100) {
      digitalWrite(INITIALIZED_LED, HIGH);
      digitalWrite(ARMED_LED      , HIGH);
      digitalWrite(RATE_LED       , HIGH);
    }
    if (currentBatteryTime > 1200) {
      previousBatteryTime = millis();
      digitalWrite(INITIALIZED_LED, LOW);
      digitalWrite(ARMED_LED      , LOW);
      digitalWrite(RATE_LED       , LOW);
    }
  }
  if (level == ALARM) {
    if (firstAlarm == OFF) autoDescent = 0; // intialize autoDescent to zero if first time in ALARM state
    firstAlarm = ON;
    digitalWrite(INITIALIZED_LED, HIGH);
    digitalWrite(ARMED_LED      , HIGH);
    digitalWrite(RATE_LED       , HIGH);
    if ((currentBatteryTime > 500) && (receiverData[THROTTLE] > 1400))
    {
      autoDescent -= 1; // auto descend quad
      previousBatteryTime = millis();
    }
  }
}

const float readBatteryVoltage(byte channel) {
  return (analogRead(channel) * batteryScaleFactor) + diode;
}

void measureBattery(byte armed) {
    batteryVoltage = filterSmooth(readBatteryVoltage(BATTERYPIN), batteryVoltage, 0.1);
    if (armed == ON) {
      if (batteryVoltage < lowVoltageWarning) batteryStatus = WARNING;
      if (batteryVoltage < lowVoltageAlarm)   batteryStatus = ALARM;
    }
    else
      batteryStatus = OK;
    
    lowBatteryEvent(batteryStatus);
  }
