/******************************************************/

void calculateMotorCommands(void)
{
  motorCommand[FRONT_LEFT]  =       (receiverData[THROTTLE] + autoDescent) + motorAxisCommand[ROLL] -       motorAxisCommand[PITCH] - motorAxisCommand[YAW];
  motorCommand[FRONT_RIGHT] =       (receiverData[THROTTLE] + autoDescent) - motorAxisCommand[ROLL] -       motorAxisCommand[PITCH] + motorAxisCommand[YAW];
  motorCommand[UPPER_REAR]  = 0.5 * (receiverData[THROTTLE] + autoDescent) - motorAxisCommand[ROLL] + 0.5 * motorAxisCommand[PITCH] - motorAxisCommand[YAW];
  motorCommand[LOWER_REAR]  = 0.5 * (receiverData[THROTTLE] + autoDescent) + motorAxisCommand[ROLL] + 0.5 * motorAxisCommand[PITCH] + motorAxisCommand[YAW];
}

/******************************************************/

void processHardManeuvers(void)
{
  // Allows multicopter to do acrobatics by lowering power to opposite motors during hard manuevers
  if (flightMode == RATE & hardManeuvers == ON) {
    if (receiverData[ROLL] < (MINCHECK - MIDCOMMAND)) {        // Maximum Left Roll Rate
      minCommand[FRONT_RIGHT] = MAXCOMMAND;
      maxCommand[FRONT_LEFT]  = minAcro;
    }
    else if (receiverData[ROLL] > (MAXCHECK - MIDCOMMAND)) {   // Maximum Right Roll Rate
      minCommand[FRONT_LEFT]  = MAXCOMMAND;
      maxCommand[FRONT_RIGHT] = minAcro;
    }
    else if (receiverData[PITCH] < (MINCHECK - MIDCOMMAND)) {  // Maximum Nose Up Pitch Rate
      minCommand[FRONT_LEFT]  = MAXCOMMAND;
      minCommand[FRONT_RIGHT] = MAXCOMMAND;
      maxCommand[UPPER_REAR]  = minAcro;
      maxCommand[LOWER_REAR]  = minAcro;
    }
    else if (receiverData[PITCH] > (MAXCHECK - MIDCOMMAND)) {  // Maximum Nose Down Pitch Rate
      minCommand[UPPER_REAR]  = MAXCOMMAND;
      minCommand[LOWER_REAR]  = MAXCOMMAND;
      maxCommand[FRONT_LEFT]  = minAcro;
      maxCommand[FRONT_RIGHT] = minAcro;
    }
  }
}

/******************************************************/
