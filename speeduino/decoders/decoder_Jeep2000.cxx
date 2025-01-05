
/** Jeep 2000 - 24 crank teeth over 720 degrees, in groups of 4 ('91 to 2000 6 cylinder Jeep engines).
* Crank wheel is high for 360 crank degrees. Quite similar to the 24X setup.
* As we only need timing within 360 degrees, only 12 tooth angles are defined.
* Tooth number 1 represents the first tooth seen after the cam signal goes high.
* www.speeduino.com/forum/download/file.php?id=205
* @defgroup dec_jeep Jeep 2000 (6 cyl)
* @{
*/
void triggerSetup_Jeep2000(void)
{
  triggerInfo.triggerToothAngle = 0; //The number of degrees that passes from tooth to tooth (primary)
  triggerInfo.toothAngles[0] = 174;
  triggerInfo.toothAngles[1] = 194;
  triggerInfo.toothAngles[2] = 214;
  triggerInfo.toothAngles[3] = 234;
  triggerInfo.toothAngles[4] = 294;
  triggerInfo.toothAngles[5] = 314;
  triggerInfo.toothAngles[6] = 334;
  triggerInfo.toothAngles[7] = 354;
  triggerInfo.toothAngles[8] = 414;
  triggerInfo.toothAngles[9] = 434;
  triggerInfo.toothAngles[10] = 454;
  triggerInfo.toothAngles[11] = 474;

  MAX_STALL_TIME = ((MICROS_PER_DEG_1_RPM/50U) * 60U); //Minimum 50rpm. (3333uS is the time per degree at 50rpm). Largest gap between teeth is 60 degrees.
  if(currentStatus.initialisationComplete == false) { triggerInfo.toothCurrentCount = 13; triggerInfo.toothLastToothTime = micros(); } //Set a startup value here to avoid filter errors when starting. This MUST have the initial check to prevent the fuel pump just staying on all the time
  BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_2ND_DERIV);
  BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_IS_SEQUENTIAL);
  BIT_SET(triggerInfo.decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);
  BIT_SET(triggerInfo.decoderState, BIT_DECODER_HAS_SECONDARY);
}

void triggerPri_Jeep2000(void)
{
  if(triggerInfo.toothCurrentCount == 13) { currentStatus.hasSync = false; } //Indicates sync has not been achieved (Still waiting for 1 revolution of the crank to take place)
  else
  {
    triggerInfo.curTime = micros();
    triggerInfo.curGap = triggerInfo.curTime - triggerInfo.toothLastToothTime;
    if ( triggerInfo.curGap >= triggerInfo.triggerFilterTime )
    {
      if(triggerInfo.toothCurrentCount == 0)
      {
         triggerInfo.toothCurrentCount = 1; //Reset the counter
         triggerInfo.toothOneMinusOneTime = triggerInfo.toothOneTime;
         triggerInfo.toothOneTime = triggerInfo.curTime;
         currentStatus.hasSync = true;
         currentStatus.startRevolutions++; //Counter
         triggerInfo.triggerToothAngle = 60; //There are groups of 4 pulses (Each 20 degrees apart), with each group being 60 degrees apart. Hence #1 is always 60
      }
      else
      {
        triggerInfo.toothCurrentCount++; //Increment the tooth counter
        triggerInfo.triggerToothAngle = triggerInfo.toothAngles[(triggerInfo.toothCurrentCount-1)] - triggerInfo.toothAngles[(triggerInfo.toothCurrentCount-2)]; //Calculate the last tooth gap in degrees
      }

      setFilter(triggerInfo.curGap); //Recalc the new filter value

      BIT_SET(triggerInfo.decoderState, BIT_DECODER_VALID_TRIGGER); //Flag this pulse as being a valid trigger (ie that it passed filters)

      triggerInfo.toothLastMinusOneToothTime = triggerInfo.toothLastToothTime;
      triggerInfo.toothLastToothTime = triggerInfo.curTime;
    } //Trigger filter
  } //Sync check
}
void triggerSec_Jeep2000(void)
{
  triggerInfo.toothCurrentCount = 0; //All we need to do is reset the tooth count back to zero, indicating that we're at the beginning of a new revolution
  return;
}

uint16_t getRPM_Jeep2000(void)
{
   return stdGetRPM(CRANK_SPEED);
}
int getCrankAngle_Jeep2000(void)
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

    int crankAngle;
    if (triggerInfo.toothCurrentCount == 0) { crankAngle = 114 + configPage4.triggerAngle; } //This is the special case to handle when the 'last tooth' seen was the cam tooth. Since  the tooth timings were taken on the previous crank tooth, the previous crank tooth angle is used here, not cam angle.
    else { crankAngle = triggerInfo.toothAngles[(temptoothCurrentCount - 1)] + configPage4.triggerAngle;} //Perform a lookup of the fixed triggerInfo.toothAngles array to find what the angle of the last tooth passed was.

    //Estimate the number of degrees travelled since the last tooth}
    triggerInfo.elapsedTime = (triggerInfo.lastCrankAngleCalc - temptoothLastToothTime);
    crankAngle += timeToAngleDegPerMicroSec(triggerInfo.elapsedTime);

    if (crankAngle >= 720) { crankAngle -= 720; }
    if (crankAngle < 0) { crankAngle += 360; }

    return crankAngle;
}

void triggerSetEndTeeth_Jeep2000(void)
{
}
/** @} */


