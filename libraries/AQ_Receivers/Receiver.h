/******************************************************/

#define TIMEOUT     25000
#define MINCOMMAND  1000
#define MIDCOMMAND  1500
#define MAXCOMMAND  2000
#define MINDELTA    200
#define MINCHECK    MINCOMMAND + 100
#define MAXCHECK    MAXCOMMAND - 100
#define MINTHROTTLE MINCOMMAND + 100
#define LEVELOFF    100

#define RISING_EDGE  1
#define FALLING_EDGE 0
#define MINONWIDTH   950
#define MAXONWIDTH   2075
#define MINOFFWIDTH  12000
#define MAXOFFWIDTH  24000

#define DEADBAND       12
#define DEADBAND_SLOPE 500/(500-DEADBAND)

/******************************************************/

#if defined(__AVR_ATmega328P__) && defined(isrSourceIsITG3200)
  #define LASTCHANNEL 5
#elif defined(__AVR_ATmega328P__)
  #define LASTCHANNEL 6
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  #define LASTCHANNEL 8
#endif

float receiverData[LASTCHANNEL];
byte  commandInDetent[LASTAXIS];
byte  previousCommandInDetent[LASTAXIS];

// Channel data
typedef struct {
  byte edge;
  unsigned long riseTime;
  unsigned long fallTime;
  unsigned int  lastGoodWidth;
} tPinTimingData;
volatile static tPinTimingData pinData[5];

#if defined(__AVR_ATmega328P__) && defined(isrSourceIsITG3200)
  static byte receiverPin[LASTCHANNEL] = {1, 2, 3, 0, 4};    // index used for ROLL, PITCH, YAW, THROTTLE, MODE
#elif defined(__AVR_ATmega328P__)
  static byte receiverPin[LASTCHANNEL] = {2, 5, 6, 4, 7, 8}; // pins used for ROLL, PITCH, YAW, THROTTLE, MODE, AUX
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  static byte receiverPin[LASTCHANNEL] = {1, 2, 3, 0, 4, 5, 6, 7}; // bit number of PORTK used for ROLL, PITCH, YAW, THROTTLE, MODE, AUX, AUX1, AUX2
#endif

/******************************************************/

void readReceiver(void)
{
  for(byte channel = ROLL; channel < LASTCHANNEL; channel++)
  {
    byte pin = receiverPin[channel];
    uint8_t oldSREG = SREG;
    cli();
    // Get receiver value read by pin change interrupt handler
    uint16_t lastGoodWidth = pinData[pin].lastGoodWidth;
    SREG = oldSREG;

    receiverData[channel] = lastGoodWidth;
  }

  receiverData[ROLL]  -= MIDCOMMAND;
  receiverData[PITCH] -= MIDCOMMAND;
  receiverData[YAW]   -= MIDCOMMAND;

  for (byte channel = FIRSTAXIS; channel < LASTAXIS; channel++)
  {
  	if ((receiverData[channel] <= DEADBAND) && (receiverData[channel] >= -DEADBAND))
    {
  	  receiverData[channel] = 0;
  	  commandInDetent[channel] = TRUE;
  	}
  	else
  	{
  	  commandInDetent[channel] = FALSE;
  	  if (receiverData[channel] > 0)
  	  {
  		receiverData[channel] = (receiverData[channel] - DEADBAND) * DEADBAND_SLOPE;
  	  }
  	  else
  	  {
  	    receiverData[channel] = (receiverData[channel] + DEADBAND) * DEADBAND_SLOPE;
  	  }
    }
  }
}

/******************************************************/
