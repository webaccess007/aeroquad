/******************************************************/

void calculateMotorCommands(void)
{
  motorCommand[FRONT] = (receiverData[THROTTLE] + autoDescent)                           - motorAxisCommand[PITCH] - motorAxisCommand[YAW];
  motorCommand[RIGHT] = (receiverData[THROTTLE] + autoDescent) - motorAxisCommand[ROLL]                            + motorAxisCommand[YAW];
  motorCommand[REAR]  = (receiverData[THROTTLE] + autoDescent)                           + motorAxisCommand[PITCH] - motorAxisCommand[YAW];
  motorCommand[LEFT]  = (receiverData[THROTTLE] + autoDescent) + motorAxisCommand[ROLL]                            + motorAxisCommand[YAW];
}

/******************************************************/

void processHardManeuvers(void)
{
  // Allows multicopter to do acrobatics by lowering power to opposite motors during hard manuevers
  if (flightMode == RATE & hardManeuvers == ON) {
    if (receiverData[ROLL] < (MINCHECK - MAXCOMMAND)) {        // Maximum Left Roll Rate
      minCommand[RIGHT] = MAXCOMMAND;
      maxCommand[LEFT]  = minAcro;
    }
    else if (receiverData[ROLL] > (MAXCHECK - MAXCOMMAND)) {   // Maximum Right Roll Rate
      minCommand[LEFT]  = MAXCOMMAND;
      maxCommand[RIGHT] = minAcro;
    }
    else if (receiverData[PITCH] < (MINCHECK - MAXCOMMAND)) {  // Maximum Nose Up Pitch Rate
      minCommand[FRONT] = MAXCOMMAND;
      maxCommand[REAR]  = minAcro;
    }
    else if (receiverData[PITCH] > (MAXCHECK - MAXCOMMAND)) {  // Maximum Nose Down Pitch Rate
      minCommand[REAR]  = MAXCOMMAND;
      maxCommand[FRONT] = minAcro;
    }
  }
}

/******************************************************/
