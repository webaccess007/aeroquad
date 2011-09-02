/* Arduino TwiMaster Library
 * Copyright (C) 2009 by William Greiman
 *
 * This file is part of the Arduino TwiMaster Library
 *
 * This Library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This Library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the Arduino TwiMaster Library.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
#ifndef TWI_MASTER_H
#define TWI_MASTER_H

#include <Wprogram.h>

#include "TWSR.h"

#define I2C_READ 1
#define I2C_WRITE 0


// I2C clock in Hz
#define F_TWI 400000L

//------------------------------------------------------------------------------
// Status codes in TWSR - names are from Atmel TWSR.h with TWSR_ added

// start condition transmitted
#define TWSR_START  0x08

// repeated start condition transmitted
#define TWSR_REP_START  0x10

// slave address plus write bit transmitted, ACK received
#define TWSR_MTX_ADR_ACK  0x18

// data transmitted, ACK received
#define TWSR_MTX_DATA_ACK  0x28

// slave address plus read bit transmitted, ACK received
#define TWSR_MRX_ADR_ACK  0x40

//------------------------------------------------------------------------------
uint8_t status_;

/** return status */
uint8_t TwiMaster_status() {
  return status_;
}

void TwiMaster_execCmd(uint8_t cmdReg)
{
  TWCR = cmdReg;
  // wait for command to complete
  byte timeout = 255;
  while (timeout-- >0 && !(TWCR & (1 << TWINT)));
	// status bits.
	status_ = TWSR & 0xF8;
  if (timeout < 2)
    TWCR = 0;
}
  

/** init hardware TWI */
void TwiMaster_init(uint8_t enablePullup)
{
  // no prescaler
  TWSR = 0;
  // set bit rate factor
  TWBR = (F_CPU/F_TWI - 16)/2;

  if (!enablePullup) return;

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  //Mega Arduino
  PORTD |= (1 << 0);
  PORTD |= (1 << 1);
#elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644__)
  // Sanguino
  PORTC |= (1 << 0);
  PORTC |= (1 << 1);
#else // __AVR_ATmega1280__
  // all other Arduinos
  PORTC |= (1 << 4);
  PORTC |= (1 << 5);
#endif // __AVR_ATmega1280__
}  

/** read byte with Ack */
uint8_t TwiMaster_readAck()
{
  TwiMaster_execCmd((1 << TWINT) | (1 << TWEN) | (1 << TWEA));
  return TWDR;
}
  
/** read byte with Nak */
uint8_t TwiMaster_readNak()
{
  TwiMaster_execCmd((1 << TWINT) | (1 << TWEN));
  return TWDR;
}
  
/** read a byte and send Ack if last is false else Nak to terminate read */
uint8_t TwiMaster_read(uint8_t last) {
  return last ? TwiMaster_readNak() : TwiMaster_readAck();
}
  
/** issue a start condition for i2c address with read/write bit */
uint8_t TwiMaster_start(uint8_t addressRW)
{
	// send START condition
	TwiMaster_execCmd((1<<TWINT) | (1<<TWSTA) | (1<<TWEN));
	if (TwiMaster_status() != TWSR_START && TwiMaster_status() != TWSR_REP_START) return 0;

	// send device address and direction
	TWDR = addressRW;
	TwiMaster_execCmd((1 << TWINT) | (1 << TWEN));
	if (addressRW & I2C_READ) {
    return TwiMaster_status() == TWSR_MRX_ADR_ACK;
  }
  else {
    return TwiMaster_status() == TWSR_MTX_ADR_ACK;
  }
}  
  
  
/** send new address and read/write bit without stop */
uint8_t TwiMaster_restart(uint8_t addressRW) {
  return TwiMaster_start(addressRW);
}
  


  

  
/** issue a stop condition */
void TwiMaster_stop()
{
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);

	// wait until stop condition is executed and bus released
  ////////////////put timeout here?
	while(TWCR & (1 << TWSTO));
}
  
/** write a byte and return true for Ack or false for Nak */
uint8_t TwiMaster_write(uint8_t data) {

	TWDR = data;
	TwiMaster_execCmd((1 << TWINT) | (1 << TWEN));
	return TwiMaster_status() == TWSR_MTX_DATA_ACK;
}



#endif //TWI_MASTER_H