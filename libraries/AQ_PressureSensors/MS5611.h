#define MS5611

#define PRESSURE_ADDRESS 0xEE

/******************************************************/

#include <PressureAltitude.h>

/******************************************************/

union {unsigned int value;
           byte bytes[2];} C1;

union {unsigned int value;
           byte bytes[2];} C2;

union {unsigned int value;
           byte bytes[2];} C3;

union {unsigned int value;
           byte bytes[2];} C4;

union {unsigned int value;
           byte bytes[2];} C5;

union {unsigned int value;
           byte bytes[2];} C6;

long dT;
long temp;

int64_t offset;
int64_t sensitivity;

long p;

/******************************************************/

void initializePressure(void) {
  delay(10);

  twiMaster.start(PRESSURE_ADDRESS | I2C_WRITE);  // Reset Device
  twiMaster.write(0x1E);
  delay(5);

  twiMaster.start(PRESSURE_ADDRESS | I2C_WRITE);  // Read Calibration Data C1
  twiMaster.write(0xA2);
  twiMaster.start(PRESSURE_ADDRESS | I2C_READ);
  C1.bytes[1] = twiMaster.read(0);
  C1.bytes[0] = twiMaster.read(1);

  twiMaster.start(PRESSURE_ADDRESS | I2C_WRITE);  // Read Calibration Data C2
  twiMaster.write(0xA4);
  twiMaster.start(PRESSURE_ADDRESS | I2C_READ);
  C2.bytes[1] = twiMaster.read(0);
  C2.bytes[0] = twiMaster.read(1);

  twiMaster.start(PRESSURE_ADDRESS | I2C_WRITE);  // Read Calibration Data C3
  twiMaster.write(0xA6);
  twiMaster.start(PRESSURE_ADDRESS | I2C_READ);
  C3.bytes[1] = twiMaster.read(0);
  C3.bytes[0] = twiMaster.read(1);

  twiMaster.start(PRESSURE_ADDRESS | I2C_WRITE);  // Read Calibration Data C4
  twiMaster.write(0xA8);
  twiMaster.start(PRESSURE_ADDRESS | I2C_READ);
  C4.bytes[1] = twiMaster.read(0);
  C4.bytes[0] = twiMaster.read(1);

  twiMaster.start(PRESSURE_ADDRESS | I2C_WRITE);  // Read Calibration Data C5
  twiMaster.write(0xAA);
  twiMaster.start(PRESSURE_ADDRESS | I2C_READ);
  C5.bytes[1] = twiMaster.read(0);
  C5.bytes[0] = twiMaster.read(1);

  twiMaster.start(PRESSURE_ADDRESS | I2C_WRITE);  // Read Calibration Data C6
  twiMaster.write(0xAC);
  twiMaster.start(PRESSURE_ADDRESS | I2C_READ);
  C6.bytes[1] = twiMaster.read(0);
  C6.bytes[0] = twiMaster.read(1);

  //  Serial.print("C1: "); Serial.println(C1);
  //  Serial.print("C2: "); Serial.println(C2);
  //  Serial.print("C3: "); Serial.println(C3);
  //  Serial.print("C4: "); Serial.println(C4);
  //  Serial.print("C5: "); Serial.println(C5);
  //  Serial.print("C6: "); Serial.println(C6);

  twiMaster.start(PRESSURE_ADDRESS | I2C_WRITE);  // Request temperature conversion
  twiMaster.write(0x58);

  delay(10);
}

/******************************************************/

void readTemperatureRequestPressure() {
  twiMaster.start(PRESSURE_ADDRESS | I2C_WRITE);  // Request temperature read
  twiMaster.write(0x00);

  twiMaster.start(PRESSURE_ADDRESS | I2C_READ);

  rawTemperature.bytes[2] = twiMaster.read(0);
  rawTemperature.bytes[1] = twiMaster.read(0);
  rawTemperature.bytes[0] = twiMaster.read(1);

  twiMaster.start(PRESSURE_ADDRESS | I2C_WRITE);  // Request pressure conversion
  twiMaster.write(0x48);
}

/******************************************************/

void readPressureRequestPressure() {
  twiMaster.start(PRESSURE_ADDRESS | I2C_WRITE);  // Request pressure read
  twiMaster.write(0x00);

  twiMaster.start(PRESSURE_ADDRESS | I2C_READ);

  rawPressure.bytes[2] = twiMaster.read(0);
  rawPressure.bytes[1] = twiMaster.read(0);
  rawPressure.bytes[0] = twiMaster.read(1);
  rawPressureSum += rawPressure.value;

  twiMaster.start(PRESSURE_ADDRESS | I2C_WRITE);  // Request pressure conversion
  twiMaster.write(0x48);
}

/******************************************************/

void readPressureRequestTemperature() {
  twiMaster.start(PRESSURE_ADDRESS | I2C_WRITE);  // Request pressure read
  twiMaster.write(0x00);

  twiMaster.start(PRESSURE_ADDRESS | I2C_READ);

  rawPressure.bytes[2] = twiMaster.read(0);
  rawPressure.bytes[1] = twiMaster.read(0);
  rawPressure.bytes[0] = twiMaster.read(1);
  rawPressureSum += rawPressure.value;

  twiMaster.start(PRESSURE_ADDRESS | I2C_WRITE);  // Request temperature conversion
  twiMaster.write(0x58);
}

/******************************************************/

void calculateTemperature()
{
  long tmpLong;

  dT = constrain((rawTemperature.value - C5.value * 256), -16776960, 16777216);
  temp = 2000 + dT * C6.value/8388608;
}

/******************************************************/

float calculatePressureAltitude() {
  offset      = constrain((C2.value * 65536 + (C4.value * dT)/128), -8589672450, 12884705280);
  sensitivity = constrain((C1.value * 32768 + (C3.value * dT)/256), -4294836225, 6442352640);
  p = (rawPressureAverage * sensitivity/2097152 - offset)/32768;

  tmpFloat = (float(p) / 101325.0);
  tmpFloat = pow(tmpFloat, 0.190295);

  return(44330 * (1.0 - tmpFloat));
}

/******************************************************/