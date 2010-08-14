/*
  AeroQuad v2.0 - July 2010
  www.AeroQuad.com
  Copyright (c) 2010 Ted Carancho.  All rights reserved.
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

class Accel {
public:
  float accelScaleFactor;
  float smoothFactor;
  int accelChannel[3];
  int accelZero[3];
  int accelData[3];
  int accelADC[3];
  int sign[3];
  byte rollChannel, pitchChannel, zAxisChannel;
  Accel(void) {
    sign[ROLL] = 1;
    sign[PITCH] = 1;
    sign[YAW] = 1;
  }

  // ******************************************************************
  // The following function calls must be defined in any new subclasses
  // ******************************************************************
  virtual void initialize(void) {
    this->_initialize(rollChannel, pitchChannel, zAxisChannel);
    smoothFactor = readFloat(ACCSMOOTH_ADR);
  }
  virtual void measure(void);
  virtual void calibrate(void);  

  // **************************************************************
  // The following functions are common between all Gyro subclasses
  // **************************************************************
  void _initialize(byte rollChannel, byte pitchChannel, byte zAxisChannel) {
    accelChannel[ROLL] = rollChannel;
    accelChannel[PITCH] = pitchChannel;
    accelChannel[ZAXIS] = zAxisChannel;
    
    accelZero[ROLL] = readFloat(LEVELROLLCAL_ADR);
    accelZero[PITCH] = readFloat(LEVELPITCHCAL_ADR);
    accelZero[ZAXIS] = readFloat(LEVELZCAL_ADR);
  }
  
  const int getRaw(byte axis) {
    return accelADC[axis] * sign[axis];
  }
  
  const int getData(byte axis) {
    return accelData[axis] * sign[axis];
  }
  
  const int invert(byte axis) {
    sign[axis] = -sign[axis];
    return sign[axis];
  }
  
  const int getZero(byte axis) {
    return accelZero[axis];
  }
  
  void setZero(byte axis, int value) {
    accelZero[axis] = value;
  }
  
  const float getScaleFactor(void) {
    return accelScaleFactor;
  }
  
  const float getSmoothFactor() {
    return smoothFactor;
  }
  
  void setSmoothFactor(float value) {
    smoothFactor = value;
  }
  
  const float angleRad(byte axis) {
    if (axis == PITCH) return arctan2(accelData[PITCH] * sign[PITCH], sqrt((long(accelData[ROLL]) * accelData[ROLL]) + (long(accelData[ZAXIS]) * accelData[ZAXIS])));
    if (axis == ROLL) return arctan2(accelData[ROLL] * sign[ROLL], sqrt((long(accelData[PITCH]) * accelData[PITCH]) + (long(accelData[ZAXIS]) * accelData[ZAXIS])));
  }

  const float angleDeg(byte axis) {
    return degrees(angleRad(axis));
  }
};

/******************************************************/
/************ AeroQuad v1 Accelerometer ***************/
/******************************************************/
#if defined(AeroQuad_v1) || defined(AeroQuadMega_v1)
class Accel_AeroQuad_v1 : public Accel {
private:
  int findZero[FINDZERO];
  
public:
  Accel_AeroQuad_v1() : Accel(){
    // Accelerometer Values
    // Update these variables if using a different accel
    // Output is ratiometric for ADXL 335
    // Note: Vs is not AREF voltage
    // If Vs = 3.6V, then output sensitivity is 360mV/g
    // If Vs = 2V, then it's 195 mV/g
    // Then if Vs = 3.3V, then it's 329.062 mV/g
    accelScaleFactor = 0.000329062;    
  }
  
  void initialize(void) {
    // rollChannel = 1
    // pitchChannel = 0
    // zAxisChannel = 2
    this->_initialize(1, 0, 2);
    smoothFactor = readFloat(ACCSMOOTH_ADR);
  }
  
  void measure(void) {
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      accelADC[axis] = analogRead(accelChannel[axis]) - accelZero[axis];
      accelData[axis] = smooth(accelADC[axis], accelData[axis], smoothFactor);
    }
  }

  // Allows user to zero accelerometers on command
  void calibrate(void) {
    for (byte calAxis = ROLL; calAxis < ZAXIS; calAxis++) {
      for (int i=0; i<FINDZERO; i++)
        findZero[i] = analogRead(accelChannel[calAxis]);
      accelZero[calAxis] = findMode(findZero, FINDZERO);
    }

    accelZero[ZAXIS] = (accelZero[ROLL] + accelZero[PITCH]) / 2;
    writeFloat(accelZero[ROLL], LEVELROLLCAL_ADR);
    writeFloat(accelZero[PITCH], LEVELPITCHCAL_ADR);
    writeFloat(accelZero[ZAXIS], LEVELZCAL_ADR);
  }
};
#endif

/******************************************************/
/********* AeroQuad Mega v2 Accelerometer *************/
/******************************************************/
#ifdef AeroQuadMega_v2
class Accel_AeroQuadMega_v2 : public Accel {
private:
  int findZero[FINDZERO];
  int accelAddress;
  int data[2];
  int rawData[3];
  byte select; // use to select which axis is being read
  
public:
  Accel_AeroQuadMega_v2() : Accel(){
    accelAddress = 0x40; // page 54 and 61 of datasheet
    // Accelerometer Values
    // Update these variables if using a different accel
    // Output is ratiometric for ADXL 335
    // Note: Vs is not AREF voltage
    // If Vs = 3.6V, then output sensitivity is 360mV/g
    // If Vs = 2V, then it's 195 mV/g
    // Then if Vs = 3.3V, then it's 329.062 mV/g
    accelScaleFactor = 0.000329062;
  }
  
  void initialize(void) {
    accelZero[ROLL] = readFloat(LEVELROLLCAL_ADR);
    accelZero[PITCH] = readFloat(LEVELPITCHCAL_ADR);
    accelZero[ZAXIS] = readFloat(LEVELZCAL_ADR);
    smoothFactor = readFloat(ACCSMOOTH_ADR);
    select = PITCH;
    
    // Check if accel is connected
    Wire.beginTransmission(accelAddress);
    Wire.send(0x00);
    Wire.endTransmission();
    delay(50);
    Wire.requestFrom(accelAddress, 1);
    data[0] = Wire.receive();
    if (data[0] != 0x03) // page 52 of datasheet
      Serial.println("Accelerometer not found!");

    // In datasheet, summary register map is page 21
    // Low pass filter settings is page 27
    // Range settings is page 28
    Wire.beginTransmission(accelAddress);
    Wire.send(0x0D);  // register ctrl_reg0
    Wire.send(0x10);  // enable writing to control registers
    Wire.endTransmission();
    
    Wire.beginTransmission(accelAddress);
    Wire.send(0x20); // register bw_tcs (bits 4-7)
    Wire.endTransmission();
    Wire.requestFrom(accelAddress, 1);
    data[0] = Wire.receive();
    Wire.beginTransmission(accelAddress);
    Wire.send(0x20);
    Wire.send(data[0] & 0x0F); // set low pass filter to 10Hz (value = 0000xxxx)
    Wire.endTransmission();

    Wire.beginTransmission(accelAddress);
    Wire.send(0x35); // register offset_lsb1 (bits 1-3)
    Wire.endTransmission();
    Wire.requestFrom(accelAddress, 1);
    data[0] = Wire.receive();
    Wire.beginTransmission(accelAddress);
    Wire.send(0x35);
    Wire.send(data[0] & ~0x0E); // set range to +/-1g (value = xxxx000x)
    Wire.endTransmission();
  }
  
  void measure(void) {
    // round robin between each axis so that I2C blocking time is low
    Wire.beginTransmission(accelAddress);
    if (select == ROLL) Wire.send(0x04);
    if (select == PITCH) Wire.send(0x02);
    if (select == ZAXIS) Wire.send(0x06);
    Wire.endTransmission();
    Wire.requestFrom(accelAddress, 2);
    data[0] = Wire.receive();
    data[1] = Wire.receive();
    rawData[select] = ((data[1] << 8) | data[0]) >> 2; // last 2 bits are not part of measurement
    accelADC[select] = (rawData[select] - accelZero[select]) >> 5; // reduce ADC value
    accelData[select] = smooth(accelADC[select], accelData[select], smoothFactor);
    if (++select == LASTAXIS) select = ROLL; // go to next axis, reset to ROLL if past ZAXIS
  }

  // Allows user to zero accelerometers on command
  void calibrate(void) {  
    int msb;
    int lsb;
    
    for (byte calAxis = ROLL; calAxis < ZAXIS; calAxis++) {
      switch(calAxis) {
        case ROLL:
          msb = 0x05;
          lsb = 0x04;
          break;
        case PITCH:
          msb = 0x03;
          lsb = 0x02;
          break;
        case ZAXIS:
          msb = 0x07;
          lsb = 0x06;
      }
      for (int i=0; i<FINDZERO; i++) {
        Wire.beginTransmission(accelAddress);
        Wire.send(msb); // request high byte
        Wire.endTransmission();
        Wire.requestFrom(accelAddress, 1);
        while (Wire.available() == 0) {/* wait for incoming data */};
        data[1] = Wire.receive();  // receive high byte (overwrites previous reading)
        data[1] = data[1] << 8;    // shift high byte to be high 8 bits
        Wire.beginTransmission(accelAddress);
        Wire.send(lsb); // request low byte
        Wire.endTransmission();
        Wire.requestFrom(accelAddress, 1);
        while (Wire.available() == 0) {/* wait for incoming data */};    
        data[0] = Wire.receive(); // receive low byte as lower 8 bits
        findZero[i] = (data[1] | data[0]) >> 2; // last two bits are not part of measurement
      }
      accelZero[calAxis] = findMode(findZero, FINDZERO);
    }

    accelZero[ZAXIS] = (accelZero[ROLL] + accelZero[PITCH]) / 2;
    writeFloat(accelZero[ROLL], LEVELROLLCAL_ADR);
    writeFloat(accelZero[PITCH], LEVELPITCHCAL_ADR);
    writeFloat(accelZero[ZAXIS], LEVELZCAL_ADR);
  }
};
#endif

/******************************************************/
/*************** APM ADC Accelerometer ****************/
/******************************************************/
#ifdef APM
class Accel_APM : public Accel {
private:
  int findZero[FINDZERO];
  int rawADC;

public:
  Accel_APM() : Accel(){
    // ADC : Voltage reference 3.3v / 12bits(4096 steps) => 0.8mV/ADC step
    // ADXL335 Sensitivity(from datasheet) => 330mV/g, 0.8mV/ADC step => 330/0.8 = 412
    // Tested value : 414
    // #define GRAVITY 414 //this equivalent to 1G in the raw data coming from the accelerometer 
    // #define Accel_Scale(x) x*(GRAVITY/9.81)//Scaling the raw data of the accel to actual acceleration in meters for seconds square
    accelScaleFactor = 414.0 / 9.81;    
  }
  
  void initialize(void) {
    // rollChannel = 5
    // pitchChannel = 4
    // zAxisChannel = 6
    this->_initialize(5, 4, 6);
  }
  
  void measure(void) {
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      rawADC = analogRead_APM_ADC(accelChannel[axis]);
      if (rawADC > 500) // Check if measurement good
        accelADC[axis] = rawADC - accelZero[axis];
      accelData[axis] = accelADC[axis]; // no smoothing needed
    }
  }

  // Allows user to zero accelerometers on command
  void calibrate(void) {
    for(byte calAxis = 0; calAxis < ZAXIS; calAxis++) {
      for (int i=0; i<FINDZERO; i++) {
        findZero[i] = analogRead_APM_ADC(accelChannel[calAxis]);
        delay(10);
      }
      accelZero[calAxis] = findMode(findZero, FINDZERO);
    }
    accelZero[ZAXIS] = (accelZero[ROLL] + accelZero[PITCH]) / 2;
    writeFloat(accelZero[ROLL], LEVELROLLCAL_ADR);
    writeFloat(accelZero[PITCH], LEVELPITCHCAL_ADR);
    writeFloat(accelZero[ZAXIS], LEVELZCAL_ADR);
  }
};
#endif

/******************************************************/
/****************** Wii Accelerometer *****************/
/******************************************************/
#ifdef AeroQuad_Wii
class Accel_Wii : public Accel {
private:
  int findZero[FINDZERO];

public:
  Accel_Wii() : Accel(){
    accelScaleFactor = 0;    
  }
  
  void initialize(void) {
    smoothFactor = readFloat(ACCSMOOTH_ADR);
  }
  
  void measure(void) {
    // Actual measurement performed in gyro class
    // We just update the appropriate variables here
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      accelADC[axis] = NWMP_acc[axis] - accelZero[axis];
      accelData[axis] = smooth(accelADC[axis], accelData[axis], smoothFactor);
    }
  }

  // Allows user to zero accelerometers on command
  void calibrate(void) {
    for(byte calAxis = ROLL; calAxis < ZAXIS; calAxis++) {
      for (int i=0; i<FINDZERO; i++) {
        updateControls();
        findZero[i] = NWMP_acc[calAxis];
      }
      accelZero[calAxis] = findMode(findZero, FINDZERO);
    }
    
    accelZero[ZAXIS] = (accelZero[ROLL] + accelZero[PITCH]) / 2;
    writeFloat(accelZero[ROLL], LEVELROLLCAL_ADR);
    writeFloat(accelZero[PITCH], LEVELPITCHCAL_ADR);
    writeFloat(accelZero[ZAXIS], LEVELZCAL_ADR);
  }
};
#endif

/******************************************************/
/************* MultiPilot Accelerometer ***************/
/******************************************************/
#if defined(Multipilot) || defined(MultipilotI2C)
class Accel_Multipilot : public Accel {
private:
  int findZero[FINDZERO];
  
public:
  Accel_Multipilot() : Accel(){
    // Accelerometer Values
    // Update these variables if using a different accel
    // Output is ratiometric for ADXL 335
    // Note: Vs is not AREF voltage
    // If Vs = 3.6V, then output sensitivity is 360mV/g
    // If Vs = 2V, then it's 195 mV/g
    // Then if Vs = 3.3V, then it's 329.062 mV/g
    // Accelerometer Values for LIS344ALH set fs to +- 2G
    // Vdd = 3.3 Volt
    // Zero = Vdd / 2
    // 3300 mV / 5  (+-2G ) = 660
    accelScaleFactor = 0.000660;
  }
  
  void initialize(void) {
    // rollChannel = 6
    // pitchChannel = 7
    // zAxisChannel = 5
    this->_initialize(6, 7, 5);
    smoothFactor = readFloat(ACCSMOOTH_ADR);
  }
  
  void measure(void) {
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      accelADC[axis] = analogRead(accelChannel[axis]) - accelZero[axis];
      accelData[axis] = smooth(accelADC[axis], accelData[axis], smoothFactor);
    }
  }

  // Allows user to zero accelerometers on command
  void calibrate(void) {
    for (byte calAxis = ROLL; calAxis < ZAXIS; calAxis++) {
      for (int i=0; i<FINDZERO; i++)
        findZero[i] = analogRead(accelChannel[calAxis]);
      accelZero[calAxis] = findMode(findZero, FINDZERO);
    }

    accelZero[ZAXIS] = (accelZero[ROLL] + accelZero[PITCH]) / 2;
    writeFloat(accelZero[ROLL], LEVELROLLCAL_ADR);
    writeFloat(accelZero[PITCH], LEVELPITCHCAL_ADR);
    writeFloat(accelZero[ZAXIS], LEVELZCAL_ADR);
  }
};
#endif