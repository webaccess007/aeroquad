/*
  AeroQuad v2.5 - November 2011
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

// This class updated by jihlein

class Compass {
public:
  float magMax[3];
  float magMin[3];
  float magCalibration[3];
  float magScale[3];
  float magOffset[3];
  float hdgX;
  float hdgY;
  int   compassAddress;
  float measuredMagX;
  float measuredMagY;
  float measuredMagZ;
  
  Compass(void) {}

  const float getHdgXY(byte axis) {
    if (axis == XAXIS) return hdgX;
    if (axis == YAXIS) return hdgY;
  }

    const int getRawData(byte axis) {
    if (axis == XAXIS) return measuredMagX;
    if (axis == YAXIS) return measuredMagY;
    if (axis == ZAXIS) return measuredMagZ;
  }
  
  void setMagCal(byte axis, float maxValue, float minValue) {
    magMax[axis] = maxValue;
    magMin[axis] = minValue;
    // Assume max/min is scaled to +1 and -1
    // y2 = 1, x2 = max; y1 = -1, x1 = min
    // m = (y2 - y1) / (x2 - x1)
    // m = 2 / (max - min)
    magScale[axis] = 2.0 / (magMax[axis] - magMin[axis]);
    // b = y1 - mx1; b = -1 - (m * min)
    magOffset[axis] = -(magScale[axis] * magMin[axis]) - 1;
  }
  
  const float getMagMax(byte axis) {
    return magMax[axis];
  }
  
  const float getMagMin(byte axis) {
    return magMin[axis];
  }
};

////////////////////////////////////////////////////////////////////////////////
// Magnetometer (HMC5843)
////////////////////////////////////////////////////////////////////////////////

class Magnetometer_HMC5843 : public Compass {
private:
  float cosRoll;
  float sinRoll;
  float cosPitch;
  float sinPitch;

public: 
  Magnetometer_HMC5843() : Compass() {
    compassAddress = 0x1E;
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Initialize HMC5843 Magnetometer
  ////////////////////////////////////////////////////////////////////////////////

  void initialize(void) {
    byte numAttempts = 0;
    bool success = false;
    delay(10);                             // Power up delay **
   
    magCalibration[XAXIS] = 1.0;
    magCalibration[YAXIS] = 1.0;
    magCalibration[ZAXIS] = 1.0;
    
    while (success == false && numAttempts < 5 ) {
      
      numAttempts++;
   
      updateRegisterI2C(compassAddress, 0x00, 0x11);  // Set positive bias configuration for sensor calibraiton
      delay(50);
   
      updateRegisterI2C(compassAddress, 0x01, 0x20); // Set +/- 1G gain
      delay(10);

      updateRegisterI2C(compassAddress, 0x02, 0x01);  // Perform single conversion
      delay(10);
   
      measure(0.0, 0.0);                    // Read calibration data
      delay(10);
   
      if ( fabs(measuredMagX) > 500.0 && fabs(measuredMagX) < 1000.0 \
          && fabs(measuredMagY) > 500.0 && fabs(measuredMagY) < 1000.0 \
          && fabs(measuredMagZ) > 500.0 && fabs(measuredMagZ) < 1000.0) {
        magCalibration[XAXIS] = fabs(715.0 / measuredMagX);
        magCalibration[YAXIS] = fabs(715.0 / measuredMagY);
        magCalibration[ZAXIS] = fabs(715.0 / measuredMagZ);
    
        success = true;
      }
   
      updateRegisterI2C(compassAddress, 0x00, 0x10);  // Set 10hz update rate and normal operaiton
      delay(50);

      updateRegisterI2C(compassAddress, 0x02, 0x00); // Continuous Update mode
      delay(50);                           // Mode change delay (1/Update Rate) **
    }

    measure(0.0, 0.0);  // Assume 1st measurement at 0 degrees roll and 0 degrees pitch
  }
  
  ////////////////////////////////////////////////////////////////////////////////
  // Measure HMC5843 Magnetometer
  ////////////////////////////////////////////////////////////////////////////////

  void measure(float roll, float pitch) {
    float magX;
    float magY;
    float tmp;
    
    sendByteI2C(compassAddress, 0x03);
    Wire.requestFrom(compassAddress, 6);

    measuredMagX =  ((Wire.receive() << 8) | Wire.receive()) * magCalibration[XAXIS];
    measuredMagY = -((Wire.receive() << 8) | Wire.receive()) * magCalibration[YAXIS];
    measuredMagZ = -((Wire.receive() << 8) | Wire.receive()) * magCalibration[ZAXIS];

    Wire.endTransmission();

    cosRoll =  cos(roll);
    sinRoll =  sin(roll);
    cosPitch = cos(pitch);
    sinPitch = sin(pitch);

    magX = ((float)measuredMagX * magScale[XAXIS] + magOffset[XAXIS]) * cosPitch + \
           ((float)measuredMagY * magScale[YAXIS] + magOffset[YAXIS]) * sinRoll * sinPitch + \
           ((float)measuredMagZ * magScale[ZAXIS] + magOffset[ZAXIS]) * cosRoll * sinPitch;
           
    magY = ((float)measuredMagY * magScale[YAXIS] + magOffset[YAXIS]) * cosRoll - \
           ((float)measuredMagZ * magScale[ZAXIS] + magOffset[ZAXIS]) * sinRoll;

    tmp  = sqrt(magX * magX + magY * magY);
    
    hdgX = magX / tmp;
    hdgY = -magY / tmp;

  }
};


////////////////////////////////////////////////////////////////////////////////
// Magnetometer (HMC5883L)
////////////////////////////////////////////////////////////////////////////////

// See HMC58x3 datasheet for more information on these values
#define NormalOperation             0x10
// Default DataOutputRate is 10hz on HMC5843 , 15hz on HMC5883L
#define DataOutputRate_Default      ( 0x04 << 2 )
#define HMC5883L_SampleAveraging_8  ( 0x03 << 5 )

class Magnetometer_HMC5883L : public Compass {
private:
  float cosRoll;
  float sinRoll;
  float cosPitch;
  float sinPitch;

public: 
  Magnetometer_HMC5883L() : Compass() {
    compassAddress = 0x1E;
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Initialize HMC5883L Magnetometer
  ////////////////////////////////////////////////////////////////////////////////

  void initialize(void) {
    byte numAttempts = 0;
    bool success = false;
    delay(10);                             // Power up delay **
   
    magCalibration[XAXIS] = 1.0;
    magCalibration[YAXIS] = 1.0;
    magCalibration[ZAXIS] = 1.0;
    
    while (success == false && numAttempts < 5 ) {
      
      numAttempts++;
   
      updateRegisterI2C(compassAddress, 0x00, 0x11);  // Set positive bias configuration for sensor calibraiton
      delay(50);
   
      updateRegisterI2C(compassAddress, 0x01, 0x20); // Set +/- 1G gain
      delay(10);

      updateRegisterI2C(compassAddress, 0x02, 0x01);  // Perform single conversion
      delay(10);
   
      measure(0.0, 0.0);                    // Read calibration data
      delay(10);
   
      if ( fabs(measuredMagX) > 500.0 && fabs(measuredMagX) < 1564.4f \
          && fabs(measuredMagY) > 500.0 && fabs(measuredMagY) < 1564.4f \
          && fabs(measuredMagZ) > 500.0 && fabs(measuredMagZ) < 1477.2f) {
        magCalibration[XAXIS] = fabs(1264.4f / measuredMagX);
        magCalibration[YAXIS] = fabs(1264.4f / measuredMagY);
        magCalibration[ZAXIS] = fabs(1177.2f / measuredMagZ); 
        success = true;
      }
   
      updateRegisterI2C(compassAddress, 0x00, HMC5883L_SampleAveraging_8 | DataOutputRate_Default | NormalOperation);
      delay(50);

      updateRegisterI2C(compassAddress, 0x02, 0x00); // Continuous Update mode
      delay(50);                           // Mode change delay (1/Update Rate) **
    }

    measure(0.0, 0.0);  // Assume 1st measurement at 0 degrees roll and 0 degrees pitch
  }
  
  ////////////////////////////////////////////////////////////////////////////////
  // Measure HMC5883L Magnetometer
  ////////////////////////////////////////////////////////////////////////////////

  void measure(float roll, float pitch) {
    float magX;
    float magY;
    float tmp;
    
    sendByteI2C(compassAddress, 0x03);
    Wire.requestFrom(compassAddress, 6);

    measuredMagY = ((Wire.receive() << 8) | Wire.receive()) * magCalibration[YAXIS];
    measuredMagZ = ((Wire.receive() << 8) | Wire.receive()) * magCalibration[ZAXIS];
    measuredMagX = ((Wire.receive() << 8) | Wire.receive()) * magCalibration[XAXIS];

    Wire.endTransmission();

    cosRoll =  cos(roll);
    sinRoll =  sin(roll);
    cosPitch = cos(pitch);
    sinPitch = sin(pitch);

    magX = ((float)measuredMagX * magScale[XAXIS] + magOffset[XAXIS]) * cosPitch + \
           ((float)measuredMagY * magScale[YAXIS] + magOffset[YAXIS]) * sinRoll * sinPitch + \
           ((float)measuredMagZ * magScale[ZAXIS] + magOffset[ZAXIS]) * cosRoll * sinPitch;
           
    magY = ((float)measuredMagY * magScale[YAXIS] + magOffset[YAXIS]) * cosRoll - \
           ((float)measuredMagZ * magScale[ZAXIS] + magOffset[ZAXIS]) * sinRoll;

    tmp  = sqrt(magX * magX + magY * magY);
    
    hdgX = magX / tmp;
    hdgY = -magY / tmp;
  }
};


// ***********************************************************************
// ************************* CHR6DM Subclass *****************************
// ***********************************************************************
#if defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
class Compass_CHR6DM : public Compass {
public:
  Compass_CHR6DM() : Compass() {}
  void initialize(void) {}
  const int getRawData(byte) {}
  void measure(float roll, float pitch) {
    heading = chr6dm.data.yaw; //this hardly needs any filtering :)
    // Change from +/-180 to 0-360
    if (heading < 0) absoluteHeading = 360 + heading;
    else absoluteHeading = heading;
  }
};

class Compass_CHR6DM_Fake : public Compass {
public:
  Compass_CHR6DM_Fake() : Compass() {}
  void initialize(void) {}
  const int getRawData(byte) {}
  void measure(float roll, float pitch) {
    heading = 0;
    // Change from +/-180 to 0-360
    if (heading < 0) absoluteHeading = 360 + heading;
    else absoluteHeading = heading;
  }
};
#endif

/*
////////////////////////////////////////////////////////////////////////////////
// Magnetometer (HMC5883L)
////////////////////////////////////////////////////////////////////////////////

#define HMC58X3_ADDR 0x1E // 7 bit address of the HMC58X3 used with the Wire library
#define HMC_POS_BIAS 1
#define HMC_NEG_BIAS 2

#define HMC58X3_R_XM 3
#define HMC58X3_R_XL 4

// HMC58X3 register map. For details see HMC58X3 datasheet
#define HMC58X3_R_CONFA 0
#define HMC58X3_R_CONFB 1
#define HMC58X3_R_MODE 2
#define HMC58X3_R_XM 3
#define HMC58X3_R_XL 4

class Magnetometer_HMC5883L : public Compass {
private:
  float cosRoll;
  float sinRoll;
  float cosPitch;
  float sinPitch;
  float magx_scale, magy_scale, magz_scale, magx_max, magy_max, magz_max;

  byte MAG_X_H;
  byte MAG_X_L;
  byte MAG_Y_H;
  byte MAG_Y_L;
  byte MAG_Z_H;
  byte MAG_Z_L;

  int magnetometer_address;
  int gain;


public: 
  Magnetometer_HMC5883L() : Compass() {
    magnetometer_address = 0x1E;
    MAG_X_H=0x03;
    MAG_X_L=0x04;
    MAG_Y_H=0x07;
    MAG_Y_L=0x08;
    MAG_Z_H=0x05;
    MAG_Z_L=0x06;
    gain = 1;
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Initialize HMC5883L Magnetometer
  ////////////////////////////////////////////////////////////////////////////////

  void initialize(void) {
    updateRegisterI2C(magnetometer_address, HMC58X3_R_CONFA, 0x70);
    updateRegisterI2C(magnetometer_address, HMC58X3_R_CONFB, 0xA0);
    updateRegisterI2C(magnetometer_address, HMC58X3_R_MODE, 0x00);

    magx_scale=1; // get actual values
    magy_scale=1;
    magz_scale=1;
    updateRegisterI2C(magnetometer_address, HMC58X3_R_CONFA, 0x010 + HMC_POS_BIAS); // Reg A DOR=0x010 + MS1,MS0 set to pos bias
  
    // set the gain
    updateRegisterI2C(magnetometer_address, HMC58X3_R_CONFB, gain << 5);
  
    // now iniit
    float x, y, z, mx=0, my=0, mz=0, t=10;
  
    byte buff[6];
  
    for (int i=0; i<(int)t; i++) { 
      updateRegisterI2C(magnetometer_address, HMC58X3_R_MODE, 1); // calibration mode
      delay(100);

      sendByteI2C(compassAddress, 0x03);
      Wire.requestFrom(compassAddress, 6);

      measuredMagX = ((Wire.receive() << 8) | Wire.receive()) * magCalibration[XAXIS];
      measuredMagZ = ((Wire.receive() << 8) | Wire.receive()) * magCalibration[ZAXIS];
      measuredMagY = ((Wire.receive() << 8) | Wire.receive()) * magCalibration[YAXIS];

      Wire.endTransmission();
    
      if (measuredMagX > mx) mx = x;
      if (measuredMagY > my) my = y;
      if (measuredMagZ > mz) mz = z;
    }
  
    float max=0;
    if (mx>max) max=mx;
    if (my>max) max=my;
    if (mz>max) max=mz;
  
    magx_max = mx;
    magy_max = my;
    magz_max = mz;
    magx_scale = max/mx; // calc scales
    magy_scale = max/my;
    magz_scale = max/mz;
    updateRegisterI2C(magnetometer_address, HMC58X3_R_CONFA, 0x010); // set RegA/DOR back to default
    delay(10);
    // now set mode
    updateRegisterI2C(magnetometer_address, HMC58X3_R_MODE, 0);
    delay(100);
  }
  
  ////////////////////////////////////////////////////////////////////////////////
  // Measure HMC5883L Magnetometer
  ////////////////////////////////////////////////////////////////////////////////

  void measure(float roll, float pitch) {
    float magX;
    float magY;
    float tmp;
    
    sendByteI2C(compassAddress, 0x03);
    Wire.requestFrom(compassAddress, 6);

    measuredMagX = ((Wire.receive() << 8) | Wire.receive()) * magCalibration[XAXIS];
    measuredMagZ = ((Wire.receive() << 8) | Wire.receive()) * magCalibration[ZAXIS];
    measuredMagY = ((Wire.receive() << 8) | Wire.receive()) * magCalibration[YAXIS];

    Wire.endTransmission();

    cosRoll =  cos(roll);
    sinRoll =  sin(roll);
    cosPitch = cos(pitch);
    sinPitch = sin(pitch);

    magX = ((float)measuredMagX * magScale[XAXIS] + magOffset[XAXIS]) * cosPitch + \
           ((float)measuredMagY * magScale[YAXIS] + magOffset[YAXIS]) * sinRoll * sinPitch + \
           ((float)measuredMagZ * magScale[ZAXIS] + magOffset[ZAXIS]) * cosRoll * sinPitch;
           
    magY = ((float)measuredMagY * magScale[YAXIS] + magOffset[YAXIS]) * cosRoll - \
           ((float)measuredMagZ * magScale[ZAXIS] + magOffset[ZAXIS]) * sinRoll;

    tmp  = sqrt(magX * magX + magY * magY);
    
    hdgX = magX / tmp;
    hdgY = -magY / tmp;

  }
};
*/