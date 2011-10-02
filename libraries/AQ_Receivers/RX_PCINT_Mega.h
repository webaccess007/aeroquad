/******************************************************/

#include <Receiver.h>

/******************************************************/

volatile uint8_t *port_to_pcmask[] = {
  &PCMSK0,
  &PCMSK1,
  &PCMSK2
};

volatile static uint8_t PCintLast[3];

/******************************************************/

ISR(PCINT2_vect, ISR_BLOCK)
{
  uint8_t bit;
  uint8_t curr;
  uint8_t mask;
  uint8_t pin;
  uint32_t currentTime;
  uint32_t time;

  curr = *portInputRegister(11);
  mask = curr ^ PCintLast[0];
  PCintLast[0] = curr;

  // mask is pins that have changed. screen out non pcint pins.
  if ((mask &= PCMSK2) == 0) {
    return;
  }

  currentTime = micros();

  // mask is pcint pins that have changed.
  for (uint8_t i=0; i < 8; i++) {
    bit = 0x01 << i;
    if (bit & mask) {
      pin = i;
      // for each pin changed, record time of change
      if (bit & PCintLast[0]) {
        time = currentTime - pinData[pin].fallTime;
        pinData[pin].riseTime = currentTime;
        if ((time >= MINOFFWIDTH) && (time <= MAXOFFWIDTH))
          pinData[pin].edge = RISING_EDGE;
        else
          pinData[pin].edge = FALLING_EDGE; // invalid rising edge detected
      }
      else {
        time = currentTime - pinData[pin].riseTime;
        pinData[pin].fallTime = currentTime;
        if ((time >= MINONWIDTH) && (time <= MAXONWIDTH) && (pinData[pin].edge == RISING_EDGE)) {
          pinData[pin].lastGoodWidth = time;
          pinData[pin].edge = FALLING_EDGE;
        }
      }
    }
  }
}

/******************************************************/

void initializeReceiver()
{
  DDRK    = 0;
  PORTK   = 0x3F;
  PCMSK2 |= 0x3F;
  PCICR  |= 0x1 << 2;

  for (byte channel = ROLL; channel < LASTCHANNEL; channel++)
  {
    pinData[receiverPin[channel]].edge = FALLING_EDGE;
  }
  pinData[0].lastGoodWidth = 1000;
  pinData[1].lastGoodWidth = 1500;
  pinData[2].lastGoodWidth = 1500;
  pinData[3].lastGoodWidth = 1500;
  pinData[4].lastGoodWidth = 1000;
  pinData[5].lastGoodWidth = 1000;
  pinData[6].lastGoodWidth = 1000;
  pinData[7].lastGoodWidth = 1000;
}

/******************************************************/
