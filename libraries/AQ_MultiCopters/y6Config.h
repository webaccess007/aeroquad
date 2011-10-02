/******************************************************/

void calculateMotorCommands(void)
{
  motorCommand[UPPER_FRONT_LEFT]  = (receiverData[THROTTLE] + autoDescent) + motorAxisCommand[ROLL] - 0.866025 * motorAxisCommand[PITCH] - motorAxisCommand[YAW];
  motorCommand[UPPER_FRONT_RIGHT] = (receiverData[THROTTLE] + autoDescent) - motorAxisCommand[ROLL] - 0.866025 * motorAxisCommand[PITCH] - motorAxisCommand[YAW];
  motorCommand[UPPER_REAR]        = (receiverData[THROTTLE] + autoDescent)                          + 0.866025 * motorAxisCommand[PITCH] + motorAxisCommand[YAW];
  motorCommand[LOWER_FRONT_LEFT]  = (receiverData[THROTTLE] + autoDescent) + motorAxisCommand[ROLL] + 0.866025 * motorAxisCommand[PITCH] + motorAxisCommand[YAW];
  motorCommand[LOWER_FRONT_RIGHT] = (receiverData[THROTTLE] + autoDescent) - motorAxisCommand[ROLL] + 0.866025 * motorAxisCommand[PITCH] + motorAxisCommand[YAW];
  motorCommand[LOWER_REAR]        = (receiverData[THROTTLE] + autoDescent)                          - 0.866025 * motorAxisCommand[PITCH] - motorAxisCommand[YAW];
}

/******************************************************/

void processHardManeuvers(void)
{
  // Allows multicopter to do acrobatics by lowering power to opposite motors during hard manuevers
  if (flightMode == RATE & hardManeuvers == ON) {
    if (receiverData[ROLL] < (MINCHECK - MIDCOMMAND)) {        // Maximum Left Roll Rate
      minCommand[UPPER_FRONT_RIGHT] = MAXCOMMAND;
      minCommand[LOWER_FRONT_RIGHT] = MAXCOMMAND;
      maxCommand[UPPER_FRONT_LEFT]  = minAcro;
      maxCommand[LOWER_FRONT_LEFT]  = minAcro;
    }
    else if (receiverData[ROLL] > (MAXCHECK - MIDCOMMAND)) {   // Maximum Right Roll Rate
      minCommand[UPPER_FRONT_LEFT]  = MAXCOMMAND;
      minCommand[LOWER_FRONT_LEFT]  = MAXCOMMAND;
      maxCommand[UPPER_FRONT_RIGHT] = minAcro;
      maxCommand[LOWER_FRONT_RIGHT] = minAcro;
    }
    else if (receiverData[PITCH] < (MINCHECK - MIDCOMMAND)) {  // Maximum Nose Up Pitch Rate
      minCommand[UPPER_FRONT_LEFT]  = MAXCOMMAND;
      minCommand[UPPER_FRONT_RIGHT] = MAXCOMMAND;
      minCommand[LOWER_FRONT_LEFT]  = MAXCOMMAND;
      maxCommand[LOWER_FRONT_RIGHT] = MAXCOMMAND;
      maxCommand[UPPER_REAR]        = minAcro;
      maxCommand[LOWER_REAR]        = minAcro;
    }
    else if (receiverData[PITCH] > (MAXCHECK - MIDCOMMAND)) {  // Maximum Nose Down Pitch Rate
      minCommand[UPPER_REAR]        = MAXCOMMAND;
      minCommand[LOWER_REAR]        = MAXCOMMAND;
      minCommand[UPPER_FRONT_LEFT]  = minAcro;
      maxCommand[UPPER_FRONT_RIGHT] = minAcro;
      maxCommand[LOWER_FRONT_LEFT]  = minAcro;
      maxCommand[LOWER_FRONT_RIGHT] = minAcro;
    }
  }
}

/******************************************************/
