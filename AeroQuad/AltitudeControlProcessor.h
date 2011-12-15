/*
  AeroQuad v3.0 - April 2011
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

// FlightControl.pde is responsible for combining sensor measurements and
// transmitter commands into motor commands for the defined flight configuration (X, +, etc.)
//////////////////////////////////////////////////////////////////////////////
/////////////////////////// calculateFlightError /////////////////////////////
//////////////////////////////////////////////////////////////////////////////

/**
 * Altitude control processor do the premilary treatment on throttle correction
 * to control the altitude of the craft. It then modify directly the 
 * throttle variable use by the motor matrix calculation
 */

#ifndef _AQ_ALTITUDE_CONTROL_PROCESSOR_H_
#define _AQ_ALTITUDE_CONTROL_PROCESSOR_H_

//rawAltitude = 44330 (1 - pow(pressure/101325.0, pressureFactor)); // returns absolute altitude in meters 
//rawAltitude = (101325.0-pressure)/4096346;  
//usAltitude = filterSmooth( constrain((float)(analogRead(A3)*0.01240),0.2,3.1), // (US voltage to meters an limited to 0.2-3.1) usAltitude,0.1);
// Apply little filtering  
//  if(usAltitude < 3.0) { 
//    altitude = usAltitude; 
//    setGroundAltitude(rawAltitude - usAltitude); 
//  } 
//  else { 
//    altitude = filterSmooth(rawAltitude - getGroundAltitude(), altitude, smoothFactor); 
//  }




//////////////////////////////////////////////////////////////////////////////
/////////////////////////// processAltitudeHold //////////////////////////////
//////////////////////////////////////////////////////////////////////////////
#if defined (AltitudeHoldBaro) && defined (AltitudeHoldRangeFinder)
  // 
  // Used both sensors
  //
  float getAltitudeFromSensors() {
    
    if (rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX] != INVALID_ALTITUDE) {
      // set the ground altitude back for a smooted sensors switch 
      baroGroundAltitude = baroRawAltitude - rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX];  
      return (rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX]); 
    }
    else {
      return getBaroAltitude();    
    }
  }
  
#elif defined (AltitudeHoldBaro) && !defined (AltitudeHoldRangeFinder)
  // 
  // Used Just Baro
  //
  float getAltitudeFromSensors() {
    return getBaroAltitude();
  }
  
#elif !defined (AltitudeHoldBaro) && defined (AltitudeHoldRangeFinder)
  // 
  // Used Just range finder
  //
  float getAltitudeFromSensors() {
    return (rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX]);
  }
  
#else
  // 
  // Used no altitude sensors
  //
  float getAltitudeFromSensors() {
    return INVALID_ALTITUDE;
  }
  
#endif



void processAltitudeHold()
{
  // ****************************** Altitude Adjust *************************
  // Thanks to Honk for his work with altitude hold
  // http://aeroquad.com/showthread.php?792-Problems-with-BMP085-I2C-barometer
  // Thanks to Sherbakov for his work in Z Axis dampening
  // http://aeroquad.com/showthread.php?359-Stable-flight-logic...&p=10325&viewfull=1#post10325
  #if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
    if (altitudeHoldState == ON) {
      float currentAltitude = getAltitudeFromSensors();
      if (currentAltitude == INVALID_ALTITUDE) {
        throttle = altitudeHoldThrottle;
        return;
      }
      int altitudeHoldThrottleCorrection = updatePID(altitudeToHoldTarget, currentAltitude, &PID[ALTITUDE_HOLD_PID_IDX]);
      altitudeHoldThrottleCorrection = constrain(altitudeHoldThrottleCorrection, minThrottleAdjust, maxThrottleAdjust);
      if (abs(altitudeHoldThrottle - receiverCommand[THROTTLE]) > altitudeHoldPanicStickMovement) {
        altitudeHoldState = ALTPANIC; // too rapid of stick movement so PANIC out of ALTHOLD
      } else {
        if (receiverCommand[THROTTLE] > (altitudeHoldThrottle + altitudeHoldBump)) { // AKA changed to use holdThrottle + ALTBUMP - (was MAXCHECK) above 1900
          altitudeToHoldTarget += 0.01;
        }
        if (receiverCommand[THROTTLE] < (altitudeHoldThrottle - altitudeHoldBump)) { // AKA change to use holdThorrle - ALTBUMP - (was MINCHECK) below 1100
          altitudeToHoldTarget -= 0.01;
        }
      }
      throttle = altitudeHoldThrottle + altitudeHoldThrottleCorrection;
    }
    else {
      throttle = receiverCommand[THROTTLE];
    }
  #else
    throttle = receiverCommand[THROTTLE];
  #endif
}



#endif // _AQ_ALTITUDE_CONTROL_PROCESSOR_H_

