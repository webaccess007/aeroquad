#define ITG3200

/******************************************************/

#ifdef AeroQuad_Mini
  #define GYRO_ADDRESS   0xD0
#else
  #define GYRO_ADDRESS   0xD2
#endif

/******************************************************/

#define LOW_PASS_FILTER 0x18  // 256 Hz Low pass filter, 8 kHz internal sample rate
//#define LOW_PASS_FILTER 0x19  // 188 Hz Low pass filter, 1 kHz internal sample rate
//#define LOW_PASS_FILTER 0x1A  //  98 Hz Low pass filter, 1 kHz internal sample rate
//#define LOW_PASS_FILTER 0x1B  //  42 Hz Low pass filter, 1 kHz internal sample rate
//#define LOW_PASS_FILTER 0x1C  //  20 Hz Low pass filter, 1 kHz internal sample rate
//#define LOW_PASS_FILTER 0x1D  //  10 Hz Low pass filter, 1 kHz internal sample rate
//#define LOW_PASS_FILTER 0x1E  //   5 Hz Low pass filter, 1 kHz internal sample rate

#ifdef ITG3200_1000HZ
  #if (LOW_PASS_FILTER == 0x18)
    #define SAMPLE_RATE_DIVISOR 0x07  // 1000 Hz = 8000/(7 + 1)
  #else
    #define SAMPLE_RATE_DIVISOR 0x00  // 1000 Hz = 1000/(0 + 1)
  #endif
#else
  #if (LOW_PASS_FILTER == 0x18)
    #define SAMPLE_RATE_DIVISOR 0x0F  // 500 Hz = 8000/(15 + 1)
  #else
    #define SAMPLE_RATE_DIVISOR 0x01  // 500 Hz = 1000/(1 + 1)
  #endif
#endif

/******************************************************/

#define gyroScaleFactor radians(1.0/14.375)  //  ITG3200 14.375 LSBs per °/sec

/******************************************************/

#include <Gyro.h>

/******************************************************/

void readGyroAndSumForAverage() {
  twiMaster.start(GYRO_ADDRESS | I2C_WRITE);
  twiMaster.write(0x1D);
  twiMaster.start(GYRO_ADDRESS | I2C_READ);

  for (byte axis = ROLL; axis < LASTAXIS; axis++) {
    rawGyro.bytes[axis*2+1] = twiMaster.read(0);
    rawGyro.bytes[axis*2]   = twiMaster.read((axis*2+1) == 5);
    gyroSum[axis] += rawGyro.value[axis];
  }
}

/******************************************************/

void computeGyroBias() {
  cli();
  for (int samples = 0; samples < 800; samples++) {
    readGyroAndSumForAverage();
    delayMicroseconds(2500);
  }

  for (byte axis = ROLL; axis < 3; axis++) {
    runTimeGyroBias[axis] = (float(gyroSum[axis])/800);
    gyroSum[axis] = 0;
  }
  sei();
}

/******************************************************/

void initializeGyro(void) {
  twiMaster.start(GYRO_ADDRESS | I2C_WRITE);  // send a reset to the device
  twiMaster.write(0x3E);
  twiMaster.write(0x80);

  twiMaster.start(GYRO_ADDRESS | I2C_WRITE);  // use internal oscillator
  twiMaster.write(0x3E);
  twiMaster.write(0x01);

  twiMaster.start(GYRO_ADDRESS | I2C_WRITE);  // internal sample rate 8000 Hz
  twiMaster.write(0x16);                      // low pass filter as defined above
  twiMaster.write(LOW_PASS_FILTER);           // +/- 2000 DPS range

  twiMaster.start(GYRO_ADDRESS | I2C_WRITE);  // sample rate divisor
  twiMaster.write(0x15);                      // as defined above
  twiMaster.write(SAMPLE_RATE_DIVISOR);

  twiMaster.start(GYRO_ADDRESS | I2C_WRITE);  // int active high
  twiMaster.write(0x17);                      // push-pull drive
  twiMaster.write(0x31);                      // latched until cleared
                                              // clear upon any register read
  delay(100);                                 // enable interrupt when data is available

  computeGyroBias();
}

/******************************************************/
