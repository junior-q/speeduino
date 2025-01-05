

/** Mazda AU version.
Tooth #2 is defined as the next crank tooth after the single cam tooth.
Tooth number one is at 348* ATDC.
* @defgroup mazda_au Mazda AU
* @{
*/
void triggerSetup_MazdaAU(void)
{
  triggerInfo.triggerToothAngle = 108; //The number of degrees that passes from tooth to tooth (primary). This is the maximum gap
  triggerInfo.toothCurrentCount = 99; //Fake tooth count represents no sync
  triggerInfo.secondaryToothCount = 0; //Needed for the cam tooth tracking
  BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_2ND_DERIV);
  BIT_SET(triggerInfo.decoderState, BIT_DECODER_IS_SEQUENTIAL);

  triggerInfo.toothAngles[0] = 348; //tooth #1
  triggerInfo.toothAngles[1] = 96; //tooth #2
  triggerInfo.toothAngles[2] = 168; //tooth #3
  triggerInfo.toothAngles[3] = 276; //tooth #4

  MAX_STALL_TIME = ((MICROS_PER_DEG_1_RPM/50U) * triggerInfo.triggerToothAngle); //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
  triggerInfo.triggerFilterTime = 1500; //10000 rpm, assuming we're triggering on both edges off the crank tooth.
  triggerInfo.triggerSecFilterTime = (int)(MICROS_PER_SEC / (MAX_RPM / 60U * 2U)) / 2U; //Same as above, but fixed at 2 teeth on the secondary input and divided by 2 (for cam speed)
  BIT_SET(triggerInfo.decoderState, BIT_DECODER_HAS_FIXED_CRANKING);
  BIT_SET(triggerInfo.decoderState, BIT_DECODER_HAS_SECONDARY);
}

void triggerPri_MazdaAU(void)
{
  triggerInfo.curTime = micros();
  triggerInfo.curGap = triggerInfo.curTime - triggerInfo.toothLastToothTime;
  if ( triggerInfo.curGap >= triggerInfo.triggerFilterTime )
  {
    BIT_SET(triggerInfo.decoderState, BIT_DECODER_VALID_TRIGGER); //Flag this pulse as being a valid trigger (ie that it passed filters)

    triggerInfo.toothCurrentCount++;
    if( (triggerInfo.toothCurrentCount == 1) || (triggerInfo.toothCurrentCount == 5) ) //Trigger is on CHANGE, hence 4 pulses = 1 crank rev
    {
       triggerInfo.toothCurrentCount = 1; //Reset the counter
       triggerInfo.toothOneMinusOneTime = triggerInfo.toothOneTime;
       triggerInfo.toothOneTime = triggerInfo.curTime;
       currentStatus.hasSync = true;
       currentStatus.startRevolutions++; //Counter
    }

    if (currentStatus.hasSync == true)
    {
      // Locked cranking timing is available, fixed at 12* BTDC
      if ( BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) && configPage4.ignCranklock )
      {
        if( triggerInfo.toothCurrentCount == 1 ) { endCoil1Charge(); }
        else if( triggerInfo.toothCurrentCount == 3 ) { endCoil2Charge(); }
      }

      //Whilst this is an uneven tooth pattern, if the specific angle between the last 2 teeth is specified, 1st deriv prediction can be used
      if( (triggerInfo.toothCurrentCount == 1) || (triggerInfo.toothCurrentCount == 3) ) { triggerInfo.triggerToothAngle = 72; triggerInfo.triggerFilterTime = triggerInfo.curGap; } //Trigger filter is set to whatever time it took to do 72 degrees (Next trigger is 108 degrees away)
      else { triggerInfo.triggerToothAngle = 108; triggerInfo.triggerFilterTime = rshift<3>(triggerInfo.curGap * 3UL); } //Trigger filter is set to (108*3)/8=40 degrees (Next trigger is 70 degrees away).

      triggerInfo.toothLastMinusOneToothTime = triggerInfo.toothLastToothTime;
      triggerInfo.toothLastToothTime = triggerInfo.curTime;
    } //Has sync
  } //Filter time
}

void triggerSec_MazdaAU(void)
{
  triggerInfo.curTime2 = micros();
  triggerInfo.lastGap = triggerInfo.curGap2;
  triggerInfo.curGap2 = triggerInfo.curTime2 - triggerInfo.toothLastSecToothTime;
  //if ( triggerInfo.curGap2 < triggerInfo.triggerSecFilterTime ) { return; }
  triggerInfo.toothLastSecToothTime = triggerInfo.curTime2;

  //if(BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) || currentStatus.hasSync == false)
  if(currentStatus.hasSync == false)
  {
    //we find sync by looking for the 2 teeth that are close together. The next crank tooth after that is the one we're looking for.
    //For the sake of this decoder, the lone cam tooth will be designated #1
    if(triggerInfo.secondaryToothCount == 2)
    {
      triggerInfo.toothCurrentCount = 1;
      currentStatus.hasSync = true;
    }
    else
    {
      triggerInfo.triggerFilterTime = 1500; //In case the engine has been running and then lost sync.
      triggerInfo.targetGap = (triggerInfo.lastGap) >> 1; //The target gap is set at half the last tooth gap
      if ( triggerInfo.curGap2 < triggerInfo.targetGap) //If the gap between this tooth and the last one is less than half of the previous gap, then we are very likely at the extra (3rd) tooth on the cam). This tooth is located at 421 crank degrees (aka 61 degrees) and therefore the last crank tooth seen was number 1 (At 350 degrees)
      {
        triggerInfo.secondaryToothCount = 2;
      }
    }
    triggerInfo.secondaryToothCount++;
  }
}


uint16_t getRPM_MazdaAU(void)
{
  uint16_t tempRPM = 0;

  if (currentStatus.hasSync == true)
  {
    //During cranking, RPM is calculated 4 times per revolution, once for each tooth on the crank signal.
    //Because these signals aren't even (Alternating 108 and 72 degrees), this needs a special function
    if(currentStatus.RPM < currentStatus.crankRPM)
    {
      int tempToothAngle;
      noInterrupts();
      tempToothAngle = triggerInfo.triggerToothAngle;
      SetRevolutionTime(36*(triggerInfo.toothLastToothTime - triggerInfo.toothLastMinusOneToothTime)); //Note that trigger tooth angle changes between 72 and 108 depending on the last tooth that was seen
      interrupts();
      tempRPM = (tempToothAngle * MICROS_PER_MIN) / revolutionTime;
    }
    else { tempRPM = stdGetRPM(CRANK_SPEED); }
  }
  return tempRPM;
}

int getCrankAngle_MazdaAU(void)
{
    int crankAngle = 0;
    if(currentStatus.hasSync == true)
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

      crankAngle = triggerInfo.toothAngles[(temptoothCurrentCount - 1)] + configPage4.triggerAngle; //Perform a lookup of the fixed triggerInfo.toothAngles array to find what the angle of the last tooth passed was.

      //Estimate the number of degrees travelled since the last tooth}
      triggerInfo.elapsedTime = (triggerInfo.lastCrankAngleCalc - temptoothLastToothTime);
      crankAngle += timeToAngleDegPerMicroSec(triggerInfo.elapsedTime);

      if (crankAngle >= 720) { crankAngle -= 720; }
      if (crankAngle < 0) { crankAngle += 360; }
    }

    return crankAngle;
}

void triggerSetEndTeeth_MazdaAU(void)
{
}
/** @} */

