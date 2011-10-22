/*
  AeroQuad v3.0 - May 2011
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

#ifndef _AQ_BATTERY_MONITOR_APM_
#define _AQ_BATTERY_MONITOR_APM_

#include "BatteryMonitorBase.h"
#include <WProgram.h>

// ***********************************************************************************
// ************************ BatteryMonitor APM & CHR6DM  *****************************
// ***********************************************************************************
/* Circuit:

  Vin--D1--R1--|--R2--GND
               |
               |
              Vout
*/
//If lipo is 12.6V and diode drop is 0.6V (res 12.0V), the voltage from divider network will be = 2.977V
//calculation: AREF/1024.0 is Vout of divider network
//Vin = lipo voltage minus the diode drop
//Vout = (Vin*R2) * (R1+R2)
//Vin = (Vout * (R1+R2))/R2
//Vin = ((((AREF/1024.0)*adDECIMAL) * (R1+R2)) / R2) + Diode drop //(aref/1024)*adDecimal is Vout
//Vout connected to Ain0 on any Arduino
/* Circuit:
  PIN57--FL_LED--150ohm--GND
  PIN58--FR_LED--150ohm--GND
  PIN59--RR_LED--150ohm--GND
  PIN60--RL_LED--150ohm--GND
*/

float diode = 0.0; //Schottky diode on APM board
float batteryScaleFactor = 0.0;

void initializeBatteryMonitor(float diodeValue) {
  float R1   = 10050; //the SMD 10k resistor measured with DMM
  float R2   =  3260; //3k3 user mounted resistor measured with DMM
  float Aref = 3.27F; //AREF 3V3 used (solder jumper) and measured with DMM
  batteryScaleFactor = ((Aref / 1024.0) * ((R1 + R2) / R2));

  diode = 0.306F; //Schottky diode on APM board, drop measured with DMM

  analogReference(EXTERNAL); //use Oilpan 3V3 AREF or if wanted, define DEFAULT here to use VCC as reference and define that voltage in BatteryReadArmLed.h

  batteryVoltage = readBatteryVoltage(BATTERYPIN);
}

const float readBatteryVoltage(byte channel) {
  return (analogRead(channel) * batteryScaleFactor) + diode;
}




#endif