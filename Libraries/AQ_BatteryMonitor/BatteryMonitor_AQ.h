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
  // below defines two batteries, ALWAYS modify this for your own configuration !!! 
  // battery 1 
  //   (vpin)     = 0   - voltage divider on v2.0 shield
  //   (cpin)     = NC  - no current sensor
  //   (vwarning) = 7.4 - warning voltage (for 2S LiPo)
  //   (valarm)   = 7.2 - alarm voltage
  //   (vscale)   = ((AREF / 1024.0) * (15.0+7.5)/7.5) - voltage divider on AQ v2.0 shield ( in - 15kOhm - out - 7.5kOhm - GND )
  //   (vbias)    = 0.82 - voltage drop on the diode on ArduinoMega VIN
  //   (cscale)   = 0.0 - N/A due to no sensor
  //   (cbias)    = 0.0 - N/A due to no sensor
 const struct BatteryConfig batConfig[] = {
   { 0, NOPIN,  BattMonitorAlarmVoltage*1.1, BattMonitorAlarmVoltage,    ((5.0 / 1024.0) * (15.0 + 7.5) / 7.5), BATTERY_MONITOR_DIODE_VALUE, 0.0, 0.0},
};
#endif