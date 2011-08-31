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

#ifndef _AEROQUAD_ACCELEROMETER_BMA180_H_
#define _AEROQUAD_ACCELEROMETER_BMA180_H_

#include <Accelerometer.h>

		



#define ACCEL_ADDRESS   0x80

/******************************************************/

void readAccel() {
  twiMaster.start(ACCEL_ADDRESS | I2C_WRITE);
  twiMaster.write(0x02);
  twiMaster.start(ACCEL_ADDRESS | I2C_READ);
  
  union {int value[3];
       byte bytes[6];} rawAccel;
  
  for (byte axis = XAXIS; axis < LASTAXIS; axis++) {
    rawAccel.bytes[axis*2]   = twiMaster.read(0);
    rawAccel.bytes[axis*2+1] = twiMaster.read((axis*2+1) == 5);
    meterPerSec[axis] = rawAccel.value[axis]>>2;
  }
}

/******************************************************/

void computeAccelBias() {

  long  accelSum[3] = {0, 0, 0};
  for (int samples = 0; samples < 800; samples++) {
    readAccel();
	for (byte axis = XAXIS; axis < LASTAXIS; axis++) {
	  accelSum[axis] += meterPerSec[axis];
	}
    delayMicroseconds(2500);
  }
  
  union {float value[3];
         byte bytes[12];} accel;
  

  for (byte axis = 0; axis < 3; axis++) {
    accel.value[axis] = (float(accelSum[axis])/800) * accelScaleFactor[axis];
  }

  runTimeAccelBias[XAXIS] = -accel.value[XAXIS];
  runTimeAccelBias[YAXIS] = -accel.value[YAXIS];
  runTimeAccelBias[ZAXIS] = -9.8065 - accel.value[ZAXIS];

  accelOneG = accel.value[ZAXIS] + runTimeAccelBias[ZAXIS];
}

/******************************************************/

void initializeAccel() {

  twiMaster.start(ACCEL_ADDRESS | I2C_WRITE);
  twiMaster.write(0x10);
  twiMaster.write(0xB6);  // Reset device

  delay(10);

  twiMaster.start(ACCEL_ADDRESS | I2C_WRITE);
  twiMaster.write(0x0D);
  twiMaster.write(0x10);  // Enable writting to control registers

  twiMaster.start(ACCEL_ADDRESS | I2C_WRITE);
  twiMaster.write(0x20);  // Register bw_tcs (bits 4-7)

  twiMaster.start(ACCEL_ADDRESS | I2C_READ);
  byte data = twiMaster.read(1);
  twiMaster.start(ACCEL_ADDRESS | I2C_WRITE);
  twiMaster.write(0x20);
  twiMaster.write(data & 0x7F);  // Set low pass filter to 1200 Hz (value = 0111xxxx)

  twiMaster.start(ACCEL_ADDRESS | I2C_WRITE);
  twiMaster.write(0x35);

  twiMaster.start(ACCEL_ADDRESS | I2C_READ);
  data = twiMaster.read(1);
  data &= 0xF1;
  data |= 0x08;

  twiMaster.start(ACCEL_ADDRESS | I2C_WRITE);
  twiMaster.write(0x35);
  twiMaster.write(data);  // Set range select bits for +/- 4g

  delay(100);

  computeAccelBias();
}

/******************************************************/

void calibrateAccel() {
  computeAccelBias();
}


#endif



/**
#include <Device_I2C.h>

#define ACCEL_ADDRESS 0x40
#define ACCEL_IDENTITY 0x03
#define ACCEL_RESET_REGISTER 0x10
#define ACCEL_TRIGER_RESET_VALUE 0xB6
#define ACCEL_ENABLE_WRITE_CONTROL_REGISTER 0x0D
#define ACCEL_CONTROL_REGISTER 0x10
#define ACCEL_BW_TCS 0x20
#define ACCEL_LOW_PASS_FILTER_REGISTER 0x20
#define ACCEL_10HZ_LOW_PASS_FILTER_VALUE 0x0F
#define ACCEL_OFFSET_REGISTER 0x35
#define ACCEL_READ_ROLL_ADDRESS 0x02
#define ACCEL_READ_PITCH_ADDRESS 0x04
#define ACCEL_READ_YAW_ADDRESS 0x06

void initializeAccel() {
  
  accelScaleFactor = G_2_MPS2(1.0/4096.0);  //  g per LSB @ +/- 2g range
  accelSmoothFactor = 1.0;

  
  if (readWhoI2C(ACCEL_ADDRESS) != ACCEL_IDENTITY) // page 52 of datasheet
    Serial.println("Accelerometer not found!");
	
  updateRegisterI2C(ACCEL_ADDRESS, ACCEL_RESET_REGISTER, ACCEL_TRIGER_RESET_VALUE); 					//reset device
  delay(10);  																							//sleep 10 ms after reset (page 25)

  // In datasheet, summary register map is page 21
  // Low pass filter settings is page 27
  // Range settings is page 28
  updateRegisterI2C(ACCEL_ADDRESS, ACCEL_ENABLE_WRITE_CONTROL_REGISTER, ACCEL_CONTROL_REGISTER); 		//enable writing to control registers
  sendByteI2C(ACCEL_ADDRESS, ACCEL_BW_TCS); 															// register bw_tcs (bits 4-7)
  byte data = readByteI2C(ACCEL_ADDRESS); 																// get current register value
  updateRegisterI2C(ACCEL_ADDRESS, ACCEL_LOW_PASS_FILTER_REGISTER, data & ACCEL_10HZ_LOW_PASS_FILTER_VALUE); 	// set low pass filter to 10Hz (value = 0000xxxx)

  // From page 27 of BMA180 Datasheet
  //  1.0g = 0.13 mg/LSB
  //  1.5g = 0.19 mg/LSB
  //  2.0g = 0.25 mg/LSB
  //  3.0g = 0.38 mg/LSB
  //  4.0g = 0.50 mg/LSB
  //  8.0g = 0.99 mg/LSB
  // 16.0g = 1.98 mg/LSB
  sendByteI2C(ACCEL_ADDRESS, ACCEL_OFFSET_REGISTER); 													// register offset_lsb1 (bits 1-3)
  data = readByteI2C(ACCEL_ADDRESS);
  data &= 0xF1;
  data |= 0x04; // Set range select bits for +/-2g
  updateRegisterI2C(ACCEL_ADDRESS, ACCEL_OFFSET_REGISTER, data);	
}

  
void measureAccel() {
  Wire.beginTransmission(ACCEL_ADDRESS);
  Wire.send(ACCEL_READ_ROLL_ADDRESS);
  Wire.endTransmission();
  Wire.requestFrom(ACCEL_ADDRESS, 6);
  
  int accelADC;
  for (byte axis = XAXIS; axis < LASTAXIS; axis++) {
    if (axis == XAXIS)
      accelADC = ((Wire.receive()|(Wire.receive() << 8)) >> 2) - accelZero[axis];
    else
      accelADC = accelZero[axis] - ((Wire.receive()|(Wire.receive() << 8)) >> 2);
    meterPerSec[axis] = filterSmooth(accelADC * accelScaleFactor, meterPerSec[axis], accelSmoothFactor);
  }  
}

void calibrateAccel() {
  int findZero[FINDZERO];
  int dataAddress;
    
  for (byte calAxis = XAXIS; calAxis < ZAXIS; calAxis++) {
    if (calAxis == XAXIS) dataAddress = ACCEL_READ_ROLL_ADDRESS;
    if (calAxis == YAXIS) dataAddress = ACCEL_READ_PITCH_ADDRESS;
    if (calAxis == ZAXIS) dataAddress = ACCEL_READ_YAW_ADDRESS;
    for (int i=0; i<FINDZERO; i++) {
      sendByteI2C(ACCEL_ADDRESS, dataAddress);
      findZero[i] = readReverseWordI2C(ACCEL_ADDRESS) >> 2; // last two bits are not part of measurement
      delay(10);
    }
    accelZero[calAxis] = findMedianInt(findZero, FINDZERO);
  }

  // replace with estimated Z axis 0g value
  accelZero[ZAXIS] = (accelZero[XAXIS] + accelZero[PITCH]) / 2;
  // store accel value that represents 1g
  measureAccel();
  accelOneG = -meterPerSec[ZAXIS];
}
*/