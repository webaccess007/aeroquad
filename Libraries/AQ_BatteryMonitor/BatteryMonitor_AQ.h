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

#ifndef _AQ_BATTERY_MONITOR_AQ_
#define _AQ_BATTERY_MONITOR_AQ_

#include "BatteryMonitor.h"
#include <WProgram.h>

void initializeBatteryMonitor(float diodeValue) {
  float R1   = 15000;
  float R2   = 7500;
  float Aref = 5.0;
  batteryScaleFactor = ((Aref / 1024.0) * ((R1 + R2) / R2));
  diode = diodeValue;
  analogReference(DEFAULT);
}

const float readBatteryVoltage(byte channel) {
  return (analogRead(channel) * batteryScaleFactor) + diode;
}


#endif