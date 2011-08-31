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


// 
// ISR (Interrupt Service Routine) for sensors here is responsible to stop the current 
// program execution and to read critical sensors, More data we get on critical sensors
// more precise is the data sampled
//


////////////////////////////////////////////////////////////////////////////////
// Interrupt Service Routine - TIMER0_COMPA
//   Due to the limitations of the 8 bit counter, this ISR fires twice,
//   for every sensor read.  Nothing of value is executed on the first fire,
//   the counter is simply reloaded.  On the second fire, the sensors are 
//   processed.  For the cost of this little overhead, we don't have to use any 
//   of the other  timing resources to generate the interrupt.  The poll for 
//   the writing of the I2C ESCs is done each time the ISR fires to minimize 
//   any phase lag issues.  The I2C ESCs are experimental and may be moved to 
//   the end of the sensor reads if later testing proves that necessary.
////////////////////////////////////////////////////////////////////////////////
ISR(TIMER0_COMPA_vect, ISR_NOBLOCK)
{  
  // If using I2C ESCs, check for updated motor commands and
  //   write them out if present.
//  #if defined(I2C_ESC)
//    if (sendMotorCommands == 1)
//    {
//      for (byte motor = FIRSTMOTOR; motor < LASTMOTOR; motor++)
//      {
//        twiMaster.start(MOTORBASE + motor | I2C_WRITE);
//        twiMaster.write(motorCommandI2C[motor]);
//      }
//      sendMotorCommands == 0;
//    }
//  #endif
    
  // Check which count has expired.  If count 1, just reload timer and return.
  // If count 2, reload timer and execute senor reads as required.  Then return.
  if (timer0countIndex == 0)        // If 1st count complete
  {
    OCR0A = TCNT0 + TIMER0_COUNT1;  // Load 2nd count
    timer0countIndex = 1;           // Indicate 2nd count active
    return;                         // And return from ISR
  }
  else {                            // Else 2nd count complete
    OCR0A = TCNT0 + TIMER0_COUNT0;  // Load 1st count
    timer0countIndex = 0;           // Indicate 1st count active
                                    // And execute sensor reads

    readAccel();
    readGyro();
    for (byte axis = 0; axis < LASTAXIS; axis++) {
      meterPerSecSum[axis] += meterPerSec[axis];
      gyroRateSum[axis] += gyroRate[axis];
    }
    gyroAccelSampleCount++;
  
    
    
//    #if defined(HMC5843) | defined(HMC5883)
//      if ((isrFrameCounter % COMPASS_COUNT) == 0) {
//        readCompass();
//        newMagData = 1;
//      }
//    #endif
    
//    #if defined(BMP085)
//      if (((isrFrameCounter + 1) % PRESSURE_COUNT) == 0) {
//        if (isrFrameCounter == (PRESSURE_COUNT-1))  readTemperatureRequestPressure();
//        else if (isrFrameCounter == (ISR_FRAME_COUNT-1)) readPressureRequestTemperature();
//        else readPressureRequestPressure();
//      }
//    #endif
    
    isrFrameCounter++;
    if (isrFrameCounter > ISR_FRAME_COUNT) isrFrameCounter = 1;
  }
}


void initIsrSensorsProcessor() {
  
  
  #if defined (__AVR_ATmega328P__)
    EICRA = 0x03;  // Set INT0 interrupt request on rising edge
    EIMSK = 0x01;  // Enable External Interrupt 0
  #endif
  
  #if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    TCCR0A = 0x00;                    // Normal port operation, OC0A, OC0B disconnected
  
    timer0countIndex = 0;
    OCR0A = TCNT0 + TIMER0_COUNT0;
    
    TIMSK0 |= (1<<OCIE0A);            // Enable Timer/Counter0 Output Compare A Match Interrupt
  #endif
  
  
//  TCCR0A = 0x00;                    // Normal port operation, OC0A, OC0B disconnected
//
//  timer0countIndex = 0;
//  OCR0A = TCNT0 + TIMER0_COUNT0;
//  
//  TIMSK0 |= (1<<OCIE0A);            // Enable Timer/Counter0 Output Compare A Match Interrupt
}


void gaterSensorsSampleSumm() {
  
  cli();  // stop isr
  
  for (byte axis = 0; axis < LASTAXIS; axis++) {
    meterPerSecSample[axis] = meterPerSecSum[axis] / gyroAccelSampleCount;
    meterPerSecSum[axis] = 0;
    gyroRateSample[axis] = gyroRateSum[axis] / gyroAccelSampleCount;
    gyroRateSum[axis] = 0;
  }
  gyroAccelSampleCount = 0;

  sei();  // restart isr
  
  meterPerSecSample[XAXIS] = meterPerSecSample[XAXIS] * accelScaleFactor[XAXIS] + runTimeAccelBias[XAXIS];
  meterPerSecSample[YAXIS] = meterPerSecSample[YAXIS] * accelScaleFactor[YAXIS] + runTimeAccelBias[YAXIS];
  meterPerSecSample[ZAXIS] = meterPerSecSample[ZAXIS] * accelScaleFactor[ZAXIS] + runTimeAccelBias[ZAXIS]; 
      
  gyroRateSample[ROLL]  = (gyroRateSample[ROLL]   - runTimeGyroBias[ROLL]) * gyroScaleFactor;
  gyroRateSample[PITCH] = (runTimeGyroBias[PITCH] - gyroRateSample[PITCH]) * gyroScaleFactor;
  gyroRateSample[YAW]   = (runTimeGyroBias[YAW]   - gyroRateSample[YAW])   * gyroScaleFactor;
}


