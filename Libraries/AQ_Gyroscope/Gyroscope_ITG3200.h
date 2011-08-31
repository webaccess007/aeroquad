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

#ifndef _AEROQUAD_GYROSCOPE_ITG3200_H_
#define _AEROQUAD_GYROSCOPE_ITG3200_H_

#include <Gyroscope.h>

//#ifdef AeroQuad_Mini_6DOF
//  #define GYRO_ADDRESS   0xD0
//#else
  #define GYRO_ADDRESS   0xD2
//#endif

/******************************************************/

#define gyroScaleFactor radians(1.0/14.375)  //  ITG3200 14.375 LSBs per °/sec

//long  gyroSum[3]  = {0, 0, 0};
//long  gyroSummedSamples[3];
float runTimeGyroBias[3];

//union {float value[3];
//        byte bytes[12];} gyro;


/******************************************************/

void readGyro() {
  twiMaster.start(GYRO_ADDRESS | I2C_WRITE);
  twiMaster.write(0x1D);
  twiMaster.start(GYRO_ADDRESS | I2C_READ);

  union {int value[3];
      byte bytes[6];} rawGyro;

  for (byte axis = ROLL; axis < LASTAXIS; axis++) {
    rawGyro.bytes[axis*2+1] = twiMaster.read(0);
    rawGyro.bytes[axis*2]   = twiMaster.read((axis*2+1) == 5);
    gyroRate[axis] = rawGyro.value[axis];
  }
}

/******************************************************/

void computeGyroBias() {

  long  gyroSum[3]  = {0, 0, 0};
  for (int samples = 0; samples < 800; samples++) {
    readGyro();
	for (byte axis = ROLL; axis < LASTAXIS; axis++) {
	  gyroSum[axis] += gyroRate[axis];
	}
    delayMicroseconds(2500);
  }

  
  
  for (byte axis = ROLL; axis < 3; axis++) {
    runTimeGyroBias[axis] = (float(gyroSum[axis])/800);
    gyroSum[axis] = 0;
  }
}

/******************************************************/

void initializeGyro() {
  twiMaster.start(GYRO_ADDRESS | I2C_WRITE);  // send a reset to the device
  twiMaster.write(0x3E);
  twiMaster.write(0x80);

  twiMaster.start(GYRO_ADDRESS | I2C_WRITE);  // use internal oscillator
  twiMaster.write(0x3E);
  twiMaster.write(0x01);

  twiMaster.start(GYRO_ADDRESS | I2C_WRITE);  // internal sample rate 8000 Hz
  twiMaster.write(0x16);                      // 256 Hz low pass filter
  twiMaster.write(0x18);                      // +/- 2000 DPS range

  twiMaster.start(GYRO_ADDRESS | I2C_WRITE);  // sample rate divisor
  twiMaster.write(0x15);                      // 500 Hz = 8000 Hz/(divider+1)
  twiMaster.write(0x0F);                      // divider = 15

  twiMaster.start(GYRO_ADDRESS | I2C_WRITE);  // int active high
  twiMaster.write(0x17);                      // push-pull drive
  twiMaster.write(0x31);                      // latched until cleared
                                              // clear upon any register read
  delay(100);                                 // enable interrupt when data is available

  computeGyroBias();
}

void calibrateGyro() {
  computeGyroBias();
}

/******************************************************/

#endif


/**
#define ITG3200_ADDRESS					0x69
#define ITG3200_MEMORY_ADDRESS			0x1D
#define ITG3200_BUFFER_SIZE				6
#define ITG3200_RESET_ADDRESS			0x3E
#define ITG3200_RESET_VALUE				0x80
#define ITG3200_LOW_PASS_FILTER_ADDR	0x16
#define ITG3200_LOW_PASS_FILTER_VALUE	0x1D	// 10Hz low pass filter
#define ITG3200_OSCILLATOR_ADDR			0x3E
#define ITG3200_OSCILLATOR_VALUE		0x01	// use X gyro oscillator
#define ITG3200_SCALE_TO_RADIANS		823.626831 // 14.375 LSBs per °/sec, / Pi / 180

int gyroAddress = ITG3200_ADDRESS;
  
//    gyroAddress = ITG3200_ADDRESS;
//    if (useSeccondAddress)
//	  gyroAddress = ITG3200_ADDRESS-1;
  

void initializeGyro() {
  gyroScaleFactor = radians(1.0 / 14.375);  //  ITG3200 14.375 LSBs per °/sec
  gyroSmoothFactor = 1.0;
  updateRegisterI2C(gyroAddress, ITG3200_RESET_ADDRESS, ITG3200_RESET_VALUE); // send a reset to the device
  updateRegisterI2C(gyroAddress, ITG3200_LOW_PASS_FILTER_ADDR, ITG3200_MEMORY_ADDRESS); // 10Hz low pass filter
  updateRegisterI2C(gyroAddress, ITG3200_RESET_ADDRESS, ITG3200_OSCILLATOR_VALUE); // use internal oscillator 
}
    
void measureGyro() {
  sendByteI2C(gyroAddress, ITG3200_MEMORY_ADDRESS);
  Wire.requestFrom(gyroAddress, ITG3200_BUFFER_SIZE);
    
  // The following 3 lines read the gyro and assign it's data to gyroADC
  // in the correct order and phase to suit the standard shield installation
  // orientation.  See TBD for details.  If your shield is not installed in this
  // orientation, this is where you make the changes.
  int gyroADC[3];
  gyroADC[ROLL]  = ((Wire.receive() << 8) | Wire.receive())  - gyroZero[ROLL];
  gyroADC[PITCH] = gyroZero[PITCH] - ((Wire.receive() << 8) | Wire.receive());
  gyroADC[YAW]   = gyroZero[YAW] - ((Wire.receive() << 8) | Wire.receive());

  for (byte axis = 0; axis <= YAW; axis++) {
    gyroRate[axis] = filterSmooth(gyroADC[axis] * gyroScaleFactor, gyroRate[axis], gyroSmoothFactor);
  }
 
  // Measure gyro heading
  long int currentTime = micros();
  if (gyroRate[YAW] > radians(1.0) || gyroRate[YAW] < radians(-1.0)) {
    heading += gyroRate[YAW] * ((currentTime - gyroLastMesuredTime) / 1000000.0);
  }
  gyroLastMesuredTime = currentTime;
}

void calibrateGyro() {
  int findZero[FINDZERO];
    
  for (byte axis = 0; axis < 3; axis++) {
    for (int i=0; i<FINDZERO; i++) {
      sendByteI2C(gyroAddress, (axis * 2) + ITG3200_LOW_PASS_FILTER_VALUE);
      findZero[i] = readWordI2C(gyroAddress);
      delay(10);
    }
    gyroZero[axis] = findMedianInt(findZero, FINDZERO);
  }
}

*/
