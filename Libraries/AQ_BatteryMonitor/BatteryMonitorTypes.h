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

#ifndef _AQ_BATTERY_MONITOR_TYPES
#define _AQ_BATTERY_MONITOR_TYPES

#define BATTERY_MONITOR_OK      0
#define BATTERY_MONITOR_WARNING 1
#define BATTERY_MONITOR_ALARM   2

struct BatteryConfig {
    byte vpin,cpin;         // A/D pins for voltage and current sensors (255 == no sensor)
    float vwarning,valarm;  // Warning and Alarm voltage level
    float vscale,vbias;     // voltage polynom V = vbias + (Aref*Ain(vpin))*vscale;
    float cscale,cbias;     // current polynom C = cbias + (Aref*Ain(cpin))*cscale;
};

#define numberOfBatteries (sizeof(batConfig)/sizeof(struct BatteryConfig))

#define NOPIN 255

#endif