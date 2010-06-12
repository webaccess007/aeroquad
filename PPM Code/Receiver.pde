/*
  AeroQuad v1.6 - March 2010
  www.AeroQuad.com
  Copyright (c) 2010 Ted Carancho.  All rights reserved.
  An Open Source Arduino based multicopter.
  
  Interrupt based method inspired by Dror Caspi
  http://www.rcgroups.com/forums/showpost.php?p=12356667&postcount=1639
  
  This program is free software: you can redistribute it and/or modify 
  it under the terms of the GNU General Public License as published by 
  the Free Software Foundation, either version 3 of the License, or 
  (at your option) any later version. 

  This program is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
  GNU General Public License for more details. 

  You should have received a copy of the GNU General Public License 
  along with this program. If not, see <http://www.gnu.org/licenses/>. 
*/

/*

PPM Recevier.pde by Maximilian Holder.
For this code you need the ServoDecode Library.
You can find it on  arduino.cc

*/

#include "Receiver.h"
#include <ServoDecode.h>



// Configure each receiver pin for PCINT
void configureReceiver() {
  pinMode(12,OUTPUT);
  pinMode(13,OUTPUT); 
  ServoDecode.begin();
  ServoDecode.setFailsafe(3,1234);
}

// Calculate PWM pulse width of receiver data
// If invalid PWM measured, use last known good time
unsigned int readReceiver(byte receiverPin) {
  
  /*
  
  My receiver has a different channel order than AeroQuad.
  This is why i change the channels.
  
  If you have a another channel order change them. The AeroQuad configurator
  helps you a lot.
  
  */
  
  switch (receiverPin)
  {
    case 0:
      receiverPin = 2;
      break;
    case 1:
      receiverPin = 3;
      break;
    case 2:
      receiverPin = 4;
      break;
    case 3:
      receiverPin = 1;
      break;
    case 4:
      receiverPin = 6;
    case 5:
      break;
  }
  
  int pulseWidth;
  pulseWidth = ServoDecode.GetChannelPulseWidth(receiverPin);
  
  return pulseWidth;
}
