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
*/

// battery 1 
//   (vpin)     = 0    - voltage divider on v2.0 shield
//   (cpin)     = NC   - no current sensor
//   (vwarning) = 10.2 - warning voltage (for 2S LiPo)
//   (valarm)   = 9.5  - alarm voltage
//   (vscale)   = ((AREF / 1024.0) * (R1+R2)/R2)
//   (vbias)    = 0.306 - voltage drop on the D1
//   (cscale)   = 0.0  - N/A due to no sensor
//   (cbias)    = 0.0  - N/A due to no sensor
const struct BatteryConfig batConfig[] = {
   { 0, NOPIN, BattMonitorAlarmVoltage*1.1, BattMonitorAlarmVoltage, ((3.27 / 1024.0) * (10.050 + 3.260) / 3.260), BATTERY_MONITOR_DIODE_VALUE, 0.0, 0.0},
}
#endif