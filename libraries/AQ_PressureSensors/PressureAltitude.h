/******************************************************/

union {unsigned long value;
           byte bytes[4];} rawTemperature;

union {unsigned long value;
           byte bytes[4];} rawPressure;

unsigned long rawPressureSum = 0;
unsigned long rawPressureSummedSamples;
unsigned long rawPressureAverage;

float tmpFloat;

union {float value;
        byte bytes[4];} pressureAltitude;

/******************************************************/