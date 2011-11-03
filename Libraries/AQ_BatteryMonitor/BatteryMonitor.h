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

#ifndef _AQ_BATTERY_MONITOR_
#define _AQ_BATTERY_MONITOR_

byte batteryStatus = BATTERY_MONITOR_OK;

float batteryVoltage[sizeof(batConfig)/sizeof(struct BatteryConfig)];
float batteryMinVoltage[sizeof(batConfig)/sizeof(struct BatteryConfig)];
float batteryCurrent[sizeof(batConfig)/sizeof(struct BatteryConfig)];
float batteryMaxCurrent[sizeof(batConfig)/sizeof(struct BatteryConfig)];
float batteryUsedCapacity[sizeof(batConfig)/sizeof(struct BatteryConfig)];

void initializeBatteryMonitor(){

  for (int i=0; i<numberOfBatteries;i++) {
    batteryVoltage[i]=batConfig[i].vwarning+1.0;
    batteryMinVoltage[i]=99.0;
    batteryCurrent[i]=0.0;
    batteryMaxCurrent[i]=0.0;
    batteryUsedCapacity[i]=0.0;
  }
}

void measureBatteryVoltage(){
  boolean alarm = false;
  boolean warning = false;
  for (int i=0; i<numberOfBatteries;i++) {
    batteryVoltage[i] = (float)analogRead(batConfig[i].vpin)*batConfig[i].vscale+batConfig[i].vbias;
    if (batteryVoltage[i]<batteryMinVoltage[i]) {
      batteryMinVoltage[i]=batteryVoltage[i];
    }
    if (batConfig[i].cpin!=NOPIN) {
      batteryCurrent[i]=  (float)analogRead(batConfig[i].cpin)*batConfig[i].cscale+batConfig[i].cbias;
      if (batteryCurrent[i]>batteryMaxCurrent[i]) { 
        batteryMaxCurrent[i]=batteryCurrent[i];
      }
      batteryUsedCapacity[i]+=1000.0*batteryCurrent[i]*G_Dt;
    }
    alarm|=(batteryVoltage[i]<batConfig[i].valarm);
    warning|=(batteryVoltage[i]<batConfig[i].vwarning);
  }
  if (alarm) {
    batteryStatus=BATTERY_MONITOR_ALARM;
  }
  else if (warning) {
    batteryStatus=BATTERY_MONITOR_WARNING;
  }
  else {
    batteryStatus=BATTERY_MONITOR_OK;
  }
}
#endif