
/** Harley Davidson (V2) with 2 unevenly Spaced Teeth.
Within the decoder code, the sync tooth is referred to as tooth #1. Derived from GMX7 and adapted for Harley.
Only rising Edge is used for simplicity.The second input is ignored, as it does not help to resolve cam position.
* @defgroup dec_harley Harley Davidson
* @{
*/
void triggerSetup_Harley(void)
{
  triggerInfo.triggerToothAngle = 0; // The number of degrees that passes from tooth to tooth, ev. 0. It alternates uneven
  BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_2ND_DERIV);
  BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_IS_SEQUENTIAL);
  BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_HAS_SECONDARY);
  MAX_STALL_TIME = ((MICROS_PER_DEG_1_RPM/50U) * 60U); //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
  if(currentStatus.initialisationComplete == false) { triggerInfo.toothLastToothTime = micros(); } //Set a startup value here to avoid filter errors when starting. This MUST have the initial check to prevent the fuel pump just staying on all the time
  triggerInfo.triggerFilterTime = 1500;
}

void triggerPri_Harley(void)
{
  triggerInfo.lastGap = triggerInfo.curGap;
  triggerInfo.curTime = micros();
  triggerInfo.curGap = triggerInfo.curTime - triggerInfo.toothLastToothTime;
  setFilter(triggerInfo.curGap); // Filtering adjusted according to setting
  if (triggerInfo.curGap > triggerInfo.triggerFilterTime)
  {
    if ( READ_PRI_TRIGGER() == HIGH) // Has to be the same as in main() trigger-attach, for readability we do it this way.
    {
        BIT_SET(triggerInfo.decoderState, BIT_DECODER_VALID_TRIGGER); //Flag this pulse as being a valid trigger (ie that it passed filters)
        triggerInfo.targetGap = triggerInfo.lastGap ; //Gap is the Time to next toothtrigger, so we know where we are
        triggerInfo.toothCurrentCount++;
        if (triggerInfo.curGap > triggerInfo.targetGap)
        {
          triggerInfo.toothCurrentCount = 1;
          triggerInfo.triggerToothAngle = 0;// Has to be equal to Angle Routine
          triggerInfo.toothOneMinusOneTime = triggerInfo.toothOneTime;
          triggerInfo.toothOneTime = triggerInfo.curTime;
          currentStatus.hasSync = true;
        }
        else
        {
          triggerInfo.toothCurrentCount = 2;
          triggerInfo.triggerToothAngle = 157;
          //     triggerInfo.toothOneMinusOneTime = triggerInfo.toothOneTime;
          //     triggerInfo.toothOneTime = triggerInfo.curTime;
        }
        triggerInfo.toothLastMinusOneToothTime = triggerInfo.toothLastToothTime;
        triggerInfo.toothLastToothTime = triggerInfo.curTime;
        currentStatus.startRevolutions++; //Counter
    }
    else
    {
      if (currentStatus.hasSync == true) { currentStatus.syncLossCounter++; }
      currentStatus.hasSync = false;
      triggerInfo.toothCurrentCount = 0;
    } //Primary trigger high
  } //Trigger filter
}


void triggerSec_Harley(void)
// Needs to be enabled in main()
{
  return;// No need for now. The only thing it could help to sync more quickly or confirm position.
} // End Sec Trigger


uint16_t getRPM_Harley(void)
{
  uint16_t tempRPM = 0;
  if (currentStatus.hasSync == true)
  {
    if ( currentStatus.RPM < (unsigned int)(configPage4.crankRPM * 100) )
    {
      // No difference with this option?
      int tempToothAngle;
      unsigned long toothTime;
      if ( (triggerInfo.toothLastToothTime == 0) || (triggerInfo.toothLastMinusOneToothTime == 0) ) { tempRPM = 0; }
      else
      {
        noInterrupts();
        tempToothAngle = triggerInfo.triggerToothAngle;
        /* High-res mode
          if(triggerInfo.toothCurrentCount == 1) { tempToothAngle = 129; }
          else { tempToothAngle = triggerInfo.toothAngles[triggerInfo.toothCurrentCount-1] - triggerInfo.toothAngles[triggerInfo.toothCurrentCount-2]; }
        */
        SetRevolutionTime(triggerInfo.toothOneTime - triggerInfo.toothOneMinusOneTime); //The time in uS that one revolution would take at current speed (The time tooth 1 was last seen, minus the time it was seen prior to that)
        toothTime = (triggerInfo.toothLastToothTime - triggerInfo.toothLastMinusOneToothTime); //Note that trigger tooth angle changes between 129 and 332 depending on the last tooth that was seen
        interrupts();
        toothTime = toothTime * 36;
        tempRPM = ((unsigned long)tempToothAngle * (MICROS_PER_MIN/10U)) / toothTime;
      }
    }
    else {
      tempRPM = stdGetRPM(CRANK_SPEED);
    }
  }
  return tempRPM;
}


int getCrankAngle_Harley(void)
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

  //Check if the last tooth seen was the reference tooth (Number 3). All others can be calculated, but tooth 3 has a unique angle
  int crankAngle;
  if ( (temptoothCurrentCount == 1) || (temptoothCurrentCount == 3) )
  {
    crankAngle = 0 + configPage4.triggerAngle; //Number of teeth that have passed since tooth 1, multiplied by the angle each tooth represents, plus the angle that tooth 1 is ATDC. This gives accuracy only to the nearest tooth.
  }
  else {
    crankAngle = 157 + configPage4.triggerAngle;
  }

  //Estimate the number of degrees travelled since the last tooth}
  triggerInfo.elapsedTime = (triggerInfo.lastCrankAngleCalc - temptoothLastToothTime);
  crankAngle += timeToAngleDegPerMicroSec(triggerInfo.elapsedTime);

  if (crankAngle >= 720) { crankAngle -= 720; }
  if (crankAngle < 0) { crankAngle += 360; }

  return crankAngle;
}

void triggerSetEndTeeth_Harley(void)
{
}
/** @} */

