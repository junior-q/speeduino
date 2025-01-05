

/** Audi with 135 teeth on the crank and 1 tooth on the cam.
* This is very similar to the dual wheel decoder, however due to the 135 teeth not dividing evenly into 360,
* only every 3rd crank tooth is used in calculating the crank angle. This effectively makes it a 45 tooth dual wheel setup.
* @defgroup dec_audi135 Audi 135
* @{
*/
void triggerSetup_Audi135(void)
{
  triggerInfo.triggerToothAngle = 8; //135/3 = 45, 360/45 = 8 degrees every 3 teeth
  triggerInfo.toothCurrentCount = UINT8_MAX; //Default value
  triggerInfo.toothSystemCount = 0;
  triggerInfo.triggerFilterTime = (unsigned long)(MICROS_PER_SEC / (MAX_RPM / 60U * 135UL)); //Trigger filter time is the shortest possible time (in uS) that there can be between crank teeth (ie at max RPM). Any pulses that occur faster than this time will be discarded as noise
  triggerInfo.triggerSecFilterTime = (int)(MICROS_PER_SEC / (MAX_RPM / 60U * 2U)) / 2U; //Same as above, but fixed at 2 teeth on the secondary input and divided by 2 (for cam speed)
  MAX_STALL_TIME = ((MICROS_PER_DEG_1_RPM/50U) * triggerInfo.triggerToothAngle); //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
  BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_2ND_DERIV);
  BIT_SET(triggerInfo.decoderState, BIT_DECODER_IS_SEQUENTIAL);
  BIT_SET(triggerInfo.decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);
  BIT_SET(triggerInfo.decoderState, BIT_DECODER_HAS_SECONDARY);
}

void triggerPri_Audi135(void)
{
   triggerInfo.curTime = micros();
   triggerInfo.curGap = triggerInfo.curTime - triggerInfo.toothSystemLastToothTime;
   if ( (triggerInfo.curGap > triggerInfo.triggerFilterTime) || (currentStatus.startRevolutions == 0) )
   {
     triggerInfo.toothSystemCount++;

     if ( currentStatus.hasSync == false ) { triggerInfo.toothLastToothTime = triggerInfo.curTime; }
     else
     {
       if ( triggerInfo.toothSystemCount >= 3 )
       {
         //We only proceed for every third tooth

         BIT_SET(triggerInfo.decoderState, BIT_DECODER_VALID_TRIGGER); //Flag this pulse as being a valid trigger (ie that it passed filters)
         triggerInfo.toothSystemLastToothTime = triggerInfo.curTime;
         triggerInfo.toothSystemCount = 0;
         triggerInfo.toothCurrentCount++; //Increment the tooth counter

         if ( (triggerInfo.toothCurrentCount == 1) || (triggerInfo.toothCurrentCount > 45) )
         {
           triggerInfo.toothCurrentCount = 1;
           triggerInfo.toothOneMinusOneTime = triggerInfo.toothOneTime;
           triggerInfo.toothOneTime = triggerInfo.curTime;
           triggerInfo.revolutionOne = !triggerInfo.revolutionOne;
           currentStatus.startRevolutions++; //Counter
         }

         setFilter(triggerInfo.curGap); //Recalc the new filter value

         triggerInfo.toothLastMinusOneToothTime = triggerInfo.toothLastToothTime;
         triggerInfo.toothLastToothTime = triggerInfo.curTime;
       } //3rd tooth check
     } // Sync check
   } // Trigger filter
}

void triggerSec_Audi135(void)
{
  /*
  triggerInfo.curTime2 = micros();
  triggerInfo.curGap2 = triggerInfo.curTime2 - triggerInfo.toothLastSecToothTime;
  if ( triggerInfo.curGap2 < triggerInfo.triggerSecFilterTime ) { return; }
  triggerInfo.toothLastSecToothTime = triggerInfo.curTime2;
  */

  if( currentStatus.hasSync == false )
  {
    triggerInfo.toothCurrentCount = 0;
    currentStatus.hasSync = true;
    triggerInfo.toothSystemCount = 3; //Need to set this to 3 so that the next primary tooth is counted
  }
  else if (configPage4.useResync == 1) { triggerInfo.toothCurrentCount = 0; triggerInfo.toothSystemCount = 3; }
  else if ( (currentStatus.startRevolutions < 100) && (triggerInfo.toothCurrentCount != 45) ) { triggerInfo.toothCurrentCount = 0; }
  triggerInfo.revolutionOne = 1; //Sequential revolution reset
}

uint16_t getRPM_Audi135(void)
{
   return stdGetRPM(CRANK_SPEED);
}

int getCrankAngle_Audi135(void)
{
    //This is the current angle ATDC the engine is at. This is the last known position based on what tooth was last 'seen'. It is only accurate to the resolution of the trigger wheel (Eg 36-1 is 10 degrees)
    unsigned long temptoothLastToothTime;
    int temptoothCurrentCount;
    bool temprevolutionOne;
    //Grab some variables that are used in the trigger code and assign them to temp variables.
    noInterrupts();
    temptoothCurrentCount = triggerInfo.toothCurrentCount;
    temptoothLastToothTime = triggerInfo.toothLastToothTime;
    temprevolutionOne = triggerInfo.revolutionOne;
    triggerInfo.lastCrankAngleCalc = micros(); //micros() is no longer interrupt safe
    interrupts();

    //Handle case where the secondary tooth was the last one seen
    if(temptoothCurrentCount == 0) { temptoothCurrentCount = 45; }

    int crankAngle = ((temptoothCurrentCount - 1) * triggerInfo.triggerToothAngle) + configPage4.triggerAngle; //Number of teeth that have passed since tooth 1, multiplied by the angle each tooth represents, plus the angle that tooth 1 is ATDC. This gives accuracy only to the nearest tooth.

    //Estimate the number of degrees travelled since the last tooth}
    triggerInfo.elapsedTime = (triggerInfo.lastCrankAngleCalc - temptoothLastToothTime);
    crankAngle += timeToAngleDegPerMicroSec(triggerInfo.elapsedTime);

    //Sequential check (simply sets whether we're on the first or 2nd revolution of the cycle)
    if (temprevolutionOne) { crankAngle += 360; }

    if (crankAngle >= 720) { crankAngle -= 720; }
    if (crankAngle < 0) { crankAngle += CRANK_ANGLE_MAX; }

    return crankAngle;
}

void triggerSetEndTeeth_Audi135(void)
{
}
/** @} */

