#include "pins_arduino.h"

volatile uint8_t *port_to_pcmask[] = {
  &PCMSK0,
  &PCMSK1,
  &PCMSK2
};

volatile static uint8_t PCintLast[3];

// Channel data 
typedef struct {   
  unsigned long riseTime;    
  unsigned long fallTime; 
  unsigned long lastGoodWidth;
} pinTimingData;  

volatile static pinTimingData pinData[24]; 

void attachPinChangeInterrupt(uint8_t pin) {
  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  uint8_t slot;
  volatile uint8_t *pcmask;

  // map pin to PCIR register
  if (port == NOT_A_PORT) {
    return;
  } 
  else {
    port -= 2;
    pcmask = port_to_pcmask[port];
  }
  // set the mask
  *pcmask |= bit;
  // enable the interrupt
  PCICR |= 0x01 << port;
}

void detachPinChangeInterrupt(uint8_t pin) {
  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  volatile uint8_t *pcmask;

  // map pin to PCIR register
  if (port == NOT_A_PORT) {
    return;
  } 
  else {
    port -= 2;
    pcmask = port_to_pcmask[port];
  }

  // disable the mask.
  *pcmask &= ~bit;
  // if that's the last one, disable the interrupt.
  if (*pcmask == 0) {
    PCICR &= ~(0x01 << port);
  }
}

static void measurePulseWidthISR(uint8_t port) {
  uint8_t bit;
  uint8_t curr;
  uint8_t mask;
  uint8_t pin;
  uint32_t currentTime;

  // get the pin states for the indicated port.
  curr = *portInputRegister(port+2);
  mask = curr ^ PCintLast[port];
  PCintLast[port] = curr;
  // mask is pins that have changed. screen out non pcint pins.
  if ((mask &= *port_to_pcmask[port]) == 0) {
    return;
  }
  currentTime = micros();
  // mask is pcint pins that have changed.
  for (uint8_t i=0; i < 8; i++) {
    bit = 0x01 << i;
    if (bit & mask) {
      pin = port * 8 + i;
      // for each pin changed, record time of change
      if (bit & PCintLast[port]) {
        pinData[pin].riseTime = currentTime;
      }
      else {
        pinData[pin].fallTime = currentTime;
      }
    }
  }
}

SIGNAL(PCINT0_vect) {
  measurePulseWidthISR(0);
}
SIGNAL(PCINT1_vect) {
  measurePulseWidthISR(1);
}
SIGNAL(PCINT2_vect) {
  measurePulseWidthISR(2);
}

#define ROLLPIN 2 
#define THROTTLEPIN 4 
#define PITCHPIN 5 
#define YAWPIN 6 
#define MODEPIN 7 
#define AUXPIN 8 
#define ROLL 0 
#define PITCH 1 
#define YAW 2 
#define THROTTLE 3 
#define MODE 4 
#define AUX 5 
#define LASTCHANNEL 6 
#define MINWIDTH 950
#define MAXWIDTH 2050
int receiverChannel[6] = {ROLLPIN, PITCHPIN, YAWPIN, THROTTLEPIN, MODEPIN, AUXPIN};
int receiverPin[6] = {18, 21, 22, 20, 23, 0};

unsigned long currentTime;
unsigned long previousTime;
byte channel;

void setup()
{
  Serial.begin(115200);
  for (channel = ROLL; channel < LASTCHANNEL; channel++) {
    pinMode(channel, INPUT);
    attachPinChangeInterrupt(receiverChannel[channel]);
  }
  previousTime = millis();
}

void loop() {
  currentTime = millis();
  if (currentTime > (previousTime + 20)) {
    for (channel = ROLL; channel < AUX; channel++) {
      Serial.print(readReceiver(receiverPin[channel]));
      Serial.print(", ");
    }
    Serial.println(readReceiver(receiverPin[AUX]));
  }
}

unsigned int readReceiver(byte receiverPin) {
  unsigned int time;
  
  time = pinData[receiverPin].fallTime - pinData[receiverPin].riseTime;
  if ((time > MINWIDTH) && (time < MAXWIDTH))
    pinData[receiverPin].lastGoodWidth = time;

  return pinData[receiverPin].lastGoodWidth;
}
    
    
