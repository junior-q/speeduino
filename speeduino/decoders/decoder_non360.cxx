


/** Non-360 Dual wheel with 2 wheels located either both on the crank or with the primary on the crank and the secondary on the cam.
There can be no missing teeth on the primary wheel.
* @defgroup dec_non360 Non-360 Dual wheel
* @{
*/
void triggerSetup_non360(void)
{
  triggerInfo.triggerToothAngle = (360U * configPage4.TrigAngMul) / configPage4.triggerTeeth; //The number of degrees that passes from tooth to tooth multiplied by the additional multiplier
  triggerInfo.toothCurrentCount = UINT8_MAX; //Default value
  triggerInfo.triggerFilterTime = (MICROS_PER_SEC / (MAX_RPM / 60U * configPage4.triggerTeeth)); //Trigger filter time is the shortest possible time (in uS) that there can be between crank teeth (ie at max RPM). Any pulses that occur faster than this time will be discarded as noise
  triggerInfo.triggerSecFilterTime = (MICROS_PER_SEC / (MAX_RPM / 60U * 2U)) / 2U; //Same as above, but fixed at 2 teeth on the secondary input and divided by 2 (for cam speed)
  BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_2ND_DERIV);
  BIT_SET(triggerInfo.decoderState, BIT_DECODER_IS_SEQUENTIAL);
  BIT_SET(triggerInfo.decoderState, BIT_DECODER_HAS_SECONDARY);
  MAX_STALL_TIME = ((MICROS_PER_DEG_1_RPM/50U) * triggerInfo.triggerToothAngle); //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
}


void triggerPri_non360(void)
{
  //This is not used, the trigger is identical to the dual wheel one, so that is used instead.
}

void triggerSec_non360(void)
{
  //This is not used, the trigger is identical to the dual wheel one, so that is used instead.
}

uint16_t getRPM_non360(void)
{
  uint16_t tempRPM = 0;
  if( (currentStatus.hasSync == true) && (triggerInfo.toothCurrentCount != 0) )
  {
    if(currentStatus.RPM < currentStatus.crankRPM) { tempRPM = crankingGetRPM(configPage4.triggerTeeth, CRANK_SPEED); }
    else { tempRPM = stdGetRPM(CRANK_SPEED); }
  }
  return tempRPM;
}

int getCrankAngle_non360(void)
{
    //This is the current angle ATDC the engine is at. This is the last known position based on what tooth was last 'seen'. It is only accurate to the resolution of the trigger wheel (Eg 36-1 is 10 degrees)
    unsigned long temptoothLastToothTime;
    int temptoothCurrentCount;
    //Grab some variables that are used in the trigger code and assign them to temp variables.
    noInterrupts();
    temptoothCurrentCount = triggerInfo.toothCurrentCount;
    temptoothLastToothTime = triggerInfo.toothLastToothTime;
    triggerInfo.lastCrankAngleCalc = micros(); //micros() is no longer interrupt safe
    interrupts();

    //Handle case where the secondary tooth was the last one seen
    if(temptoothCurrentCount == 0) { temptoothCurrentCount = configPage4.triggerTeeth; }

    //Number of teeth that have passed since tooth 1, multiplied by the angle each tooth represents, plus the angle that tooth 1 is ATDC. This gives accuracy only to the nearest tooth.
    int crankAngle = (temptoothCurrentCount - 1) * triggerInfo.triggerToothAngle;
    crankAngle = (crankAngle / configPage4.TrigAngMul) + configPage4.triggerAngle; //Have to divide by the multiplier to get back to actual crank angle.

    //Estimate the number of degrees travelled since the last tooth}
    triggerInfo.elapsedTime = (triggerInfo.lastCrankAngleCalc - temptoothLastToothTime);
    crankAngle += timeToAngleDegPerMicroSec(triggerInfo.elapsedTime);

    if (crankAngle >= 720) { crankAngle -= 720; }
    if (crankAngle < 0) { crankAngle += 360; }

    return crankAngle;
}

void triggerSetEndTeeth_non360(void)
{
}
/** @} */


