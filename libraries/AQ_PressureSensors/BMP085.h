#define BMP085

#define PRESSURE_ADDRESS 0xEE

#define OSS 2

/******************************************************/

#include <PressureAltitude.h>

/******************************************************/

union { int value;
       byte bytes[2];} ac1;

union { int value;
       byte bytes[2];} ac2;

union { int value;
       byte bytes[2];} ac3;

union { int value;
       byte bytes[2];} b1;

union { int value;
       byte bytes[2];} b2;

union { int value;
       byte bytes[2];} mb;

union { int value;
       byte bytes[2];} mc;

union { int value;
       byte bytes[2];} md;

union {unsigned int value;
               byte bytes[2];} ac4;

union {unsigned int value;
               byte bytes[2];} ac5;

union {unsigned int value;
               byte bytes[2];} ac6;

long b3, b5, b6;
long p;
long x1, x2, x3;
unsigned long b4, b7;
long tmp;

/******************************************************/

void initializePressure(void) {
  delay(10);

  twiMaster.start(PRESSURE_ADDRESS | I2C_WRITE);  // Read calibration data
  twiMaster.write(0xAA);
  twiMaster.start(PRESSURE_ADDRESS | I2C_READ);

  ac1.bytes[1] = twiMaster.read(0);  ac1.bytes[0] = twiMaster.read(0);
  ac2.bytes[1] = twiMaster.read(0);  ac2.bytes[0] = twiMaster.read(0);
  ac3.bytes[1] = twiMaster.read(0);  ac3.bytes[0] = twiMaster.read(0);
  ac4.bytes[1] = twiMaster.read(0);  ac4.bytes[0] = twiMaster.read(0);
  ac5.bytes[1] = twiMaster.read(0);  ac5.bytes[0] = twiMaster.read(0);
  ac6.bytes[1] = twiMaster.read(0);  ac6.bytes[0] = twiMaster.read(0);
  b1.bytes[1]  = twiMaster.read(0);  b1.bytes[0]  = twiMaster.read(0);
  b2.bytes[1]  = twiMaster.read(0);  b2.bytes[0]  = twiMaster.read(0);
  mb.bytes[1]  = twiMaster.read(0);  mb.bytes[0]  = twiMaster.read(0);
  mc.bytes[1]  = twiMaster.read(0);  mc.bytes[0]  = twiMaster.read(0);
  md.bytes[1]  = twiMaster.read(0);  md.bytes[0]  = twiMaster.read(1);

//  Serial.print("ac1: "); Serial.println(ac1);
//  Serial.print("ac2: "); Serial.println(ac2);
//  Serial.print("ac3: "); Serial.println(ac3);
//  Serial.print("ac4: "); Serial.println(ac4);
//  Serial.print("ac5: "); Serial.println(ac5);
//  Serial.print("ac6: "); Serial.println(ac6);
//  Serial.print(" b1: "); Serial.println(b1);
//  Serial.print(" b2: "); Serial.println(b2);
//  Serial.print(" mb: "); Serial.println(mb);
//  Serial.print(" mc: "); Serial.println(mc);
//  Serial.print(" md: "); Serial.println(md);

  twiMaster.start(PRESSURE_ADDRESS | I2C_WRITE);  // Request temperature read
  twiMaster.write(0xF4);
  twiMaster.write(0x2E);
  delay(5);
}

/******************************************************/

void readTemperatureRequestPressure() {
  twiMaster.start(PRESSURE_ADDRESS | I2C_WRITE);  // Set temperature read address
  twiMaster.write(0xF6);
  twiMaster.start(PRESSURE_ADDRESS | I2C_READ);

  rawTemperature.bytes[1] = twiMaster.read(0);
  rawTemperature.bytes[0] = twiMaster.read(1);

  twiMaster.start(PRESSURE_ADDRESS | I2C_WRITE);  // Request pressure read
  twiMaster.write(0xF4);
  twiMaster.write(0x34 + (OSS<<6));
}

/******************************************************/

void readPressureRequestPressure() {
  twiMaster.start(PRESSURE_ADDRESS | I2C_WRITE);  // Set pressure read address
  twiMaster.write(0xF6);
  twiMaster.start(PRESSURE_ADDRESS | I2C_READ);

  rawPressure.bytes[2] = twiMaster.read(0);
  rawPressure.bytes[1] = twiMaster.read(0);
  rawPressure.bytes[0] = twiMaster.read(1);
  rawPressureSum += rawPressure.value;

  twiMaster.start(PRESSURE_ADDRESS | I2C_WRITE);  // Request pressure read
  twiMaster.write(0xF4);
  twiMaster.write(0x34 + (OSS<<6));
}

/******************************************************/

void readPressureRequestTemperature() {
  twiMaster.start(PRESSURE_ADDRESS | I2C_WRITE);  // Set pressure read address
  twiMaster.write(0xF6);
  twiMaster.start(PRESSURE_ADDRESS | I2C_READ);

  rawPressure.bytes[2] = twiMaster.read(0);
  rawPressure.bytes[1] = twiMaster.read(0);
  rawPressure.bytes[0] = twiMaster.read(1);
  rawPressureSum += rawPressure.value;

  twiMaster.start(PRESSURE_ADDRESS | I2C_WRITE);  // Request temperature read
  twiMaster.write(0xF4);
  twiMaster.write(0x2E);
}

/******************************************************/

void calculateTemperature() {
  x1 = (rawTemperature.value - ac6.value) * ac5.value / 32768;
  x2 = ((long)mc.value * 2048) / (x1 + md.value);
  b5 = x1 + x2;
}

/******************************************************/

float calculatePressureAltitude() {
  b6 = b5 - 4000;
  x1 = (b2.value * (b6 * b6 >> 12)) / 2048;
  x2 = ac2.value * b6 / 2048;
  x3 = x1 + x2;
  tmp = (ac1.value * 4 + x3) << OSS;
  b3 = (tmp + 2) >> 2;
  x1 = ac3.value * b6 >> 13;
  x2 = (b1.value * (b6 * b6 >> 12)) / 65536;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (ac4.value * (long)(x3 + 32768)) / 32768;
  b7 = ((long)(rawPressureAverage >> (8-OSS)) - b3) * (50000 >> OSS);
  p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;
  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) / 65536;
  x2 = (-7357 * p) / 65536;
  p = p + ((x1 + x2 + 3791) / 16);

  tmpFloat = (p / 101325.0);
  tmpFloat = pow(tmpFloat, 0.190295);

  return(44330 * (1.0 -tmpFloat));
}

/******************************************************/