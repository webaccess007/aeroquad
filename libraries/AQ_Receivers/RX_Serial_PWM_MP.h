/******************************************************/

#define MINCOMMAND  2000
#define MIDCOMMAND  3000
#define MAXCOMMAND  4000
#define MINCHECK    MINCOMMAND + 200
#define MAXCHECK    MAXCOMMAND - 200
#define MINTHROTTLE MINCOMMAND + 200

#define MIN_PULSE_ON_WIDTH 1800
#define MAX_PULSE_ON_WIDTH 4200
#define PWM_GAP_TIME        200

#define DEADBAND       24
#define DEADBAND_SLOPE 1000/(1000-DEADBAND)

#define FIRSTCHANNEL 0
#define  LASTCHANNEL 8

float receiverData[LASTCHANNEL];
byte  commandInDetent[LASTAXIS];
byte  previousCommandInDetent[LASTAXIS];

volatile unsigned int  ICR5_old;
volatile unsigned char pwmCounter=0;
volatile unsigned int  pwmRaw[8] = {3000,3000,3000,2000,2000,2000,2000,2000};

/******************************************************/

int getChannel(byte ch)
{
  int result1;
  int result2;

  // Because servo pulse variables are 16 bits and the interrupts are running values could be corrupted.
  // We dont want to stop interrupts to read radio channels so we have to do two readings to be sure that the value is correct...
  result1 =  pwmRaw[ch];
  result2 =  pwmRaw[ch];
  if (result1 != result2)
    result1 =  pwmRaw[ch];  // if the results are different we make a third reading (this should be fine)

  // Limit values to a valid range
  result1 = constrain(result1,MIN_PULSE_ON_WIDTH,MAX_PULSE_ON_WIDTH);
  return(result1);
}

/******************************************************/

void readReceiver(void)
{
  for(byte channel = ROLL; channel < LASTCHANNEL; channel++)
  {
    receiverData[channel] = getChannel(channel);
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

void initializeReceiver(void)
{
  pinMode(48, INPUT);                            // ICP5 pin (PL1) (PPM input)

  TCCR5B = 0x00;                                 // Timer 5 Off

  TCCR5A = (1<<WGM50) |(1<<WGM51)|
           (1<<COM5C1)|(1<<COM5B1)|(1<<COM5A1);


  TCCR5B = (1<<WGM53)|(1<<WGM52)|
           (1<<ICES5);                           // Input Capture rising edge

  OCR5A = 40000;                                 // 50hz frequency
  OCR5B = 3000;                                  // PL4, D45
  OCR5C = 3000;                                  // PL5, D44

  TIMSK5 |= (1<<ICIE5);                          // Enable Input Capture interrupt
  TCCR5B |= (1<<CS51);                           // Timer 5 Prescaler = divide by 8
}

/******************************************************/

ISR(TIMER5_CAPT_vect)
{
  unsigned int pulse;
  unsigned int pulseWidth;

  pulse=ICR5;
  if (pulse<ICR5_old)                            // Take care of the overflow of Timer4 (TOP=40000)
    pulseWidth=(pulse + 40000)-ICR5_old;         // Calculating pulse
  else
    pulseWidth=pulse-ICR5_old;                   // Calculating pulse
  if (pulseWidth>4500)                           // SYNC pulse?
    pwmCounter=0;
  else
    {
      pwmCounter &= 0x07;                        // For safety only (limit PPM_Counter to 7)
      pwmRaw[pwmCounter++]=pulseWidth;           // Saving pulse.
    }
  ICR5_old = pulse;
}

/******************************************************/