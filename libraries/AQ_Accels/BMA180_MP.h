#define BMA180

#define ACCEL_ADDRESS   0x80

/******************************************************/

//#define BANDWIDTH 0x0F  //   10 Hz bandwidth
//#define BANDWIDTH 0x1F  //   20 Hz bandwidth
//#define BANDWIDTH 0x2F  //   40 Hz bandwidth
//#define BANDWIDTH 0x3F  //   75 Hz bandwidth
//#define BANDWIDTH 0x4F  //  150 Hz bandwidth
//#define BANDWIDTH 0x5F  //  300 Hz bandwidth
//#define BANDWIDTH 0x6F  //  600 Hz bandwidth
#define BANDWIDTH 0x7F  // 1200 Hz bandwidth

/******************************************************/

#include <Accel_MP.h>

/******************************************************/

void readAccelAndSumForAverage() {
  twiMaster.start(ACCEL_ADDRESS | I2C_WRITE);
  twiMaster.write(0x02);
  twiMaster.start(ACCEL_ADDRESS | I2C_READ);

  for (byte axis = XAXIS; axis < LASTAXIS; axis++) {
    rawAccel.bytes[axis*2]   = twiMaster.read(0);
    rawAccel.bytes[axis*2+1] = twiMaster.read((axis*2+1) == 5);
    rawAccel.value[axis] = rawAccel.value[axis]>>2;
    accelSum[axis] += rawAccel.value[axis];
  }
}

/******************************************************/

void computeAccelBias() {
  cli();
  for (int samples = 0; samples < 2000; samples++) {
    readAccelAndSumForAverage();
    delayMicroseconds(1000);
  }

  for (byte axis = 0; axis < 3; axis++) {
    accel.value[axis] = (float(accelSum[axis])/2000) * accelScaleFactor[axis];
    accelSum[axis] = 0;
  }

  runTimeAccelBias[XAXIS] = -accel.value[XAXIS];
  runTimeAccelBias[YAXIS] = -accel.value[YAXIS];
  runTimeAccelBias[ZAXIS] = -9.8065 - accel.value[ZAXIS];

  oneG.value = abs(accel.value[ZAXIS] + runTimeAccelBias[ZAXIS]);
  sei();
}

/******************************************************/

void initializeAccel(void) {
  byte data;

  twiMaster.start(ACCEL_ADDRESS | I2C_WRITE);
  twiMaster.write(0x10);
  twiMaster.write(0xB6);  // Reset device

  delay(10);

  twiMaster.start(ACCEL_ADDRESS | I2C_WRITE);
  twiMaster.write(0x0D);
  twiMaster.write(0x10);  // Enable writting to control registers

  twiMaster.start(ACCEL_ADDRESS | I2C_WRITE);
  twiMaster.write(0x20);  // Register bw_tcs

  twiMaster.start(ACCEL_ADDRESS | I2C_READ);
  data = twiMaster.read(1);
  twiMaster.start(ACCEL_ADDRESS | I2C_WRITE);
  twiMaster.write(0x20);
  twiMaster.write(data & BANDWIDTH);  // Set low pass filter to bandwith selected above

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

