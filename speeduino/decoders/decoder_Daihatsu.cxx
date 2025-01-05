

/** Daihatsu +1 trigger for 3 and 4 cylinder engines.
* Tooth equal to the number of cylinders are evenly spaced on the cam. No position sensing (Distributor is retained),
* so crank angle is a made up figure based purely on the first teeth to be seen.
* Note: This is a very simple decoder. See http://www.megamanual.com/ms2/GM_7pinHEI.htm
* @defgroup dec_daihatsu Daihatsu (3  and 4 cyl.)
* @{
*/
void triggerSetup_Daihatsu(void)
{
  triggerInfo.triggerActualTeeth = configPage2.nCylinders + 1;
  triggerInfo.triggerToothAngle = 720 / triggerInfo.triggerActualTeeth; //The number of degrees that passes from tooth to tooth
  triggerInfo.triggerFilterTime = MICROS_PER_MIN / MAX_RPM / configPage2.nCylinders; // Minimum time required between teeth
  triggerInfo.triggerFilterTime = triggerInfo.triggerFilterTime / 2; //Safety margin
  BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_2ND_DERIV);
  BIT_SET(triggerInfo.decoderState, BIT_DECODER_IS_SEQUENTIAL);
  BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_HAS_SECONDARY);

  MAX_STALL_TIME = ((MICROS_PER_DEG_1_RPM/90U) * triggerInfo.triggerToothAngle)*4U;//Minimum 90rpm. (1851uS is the time per degree at 90rpm). This uses 90rpm rather than 50rpm due to the potentially very high stall time on a 4 cylinder if we wait that long.

  if(configPage2.nCylinders == 3)
  {
    triggerInfo.toothAngles[0] = 0; //tooth #1
    triggerInfo.toothAngles[1] = 30; //tooth #2 (Extra tooth)
    triggerInfo.toothAngles[2] = 240; //tooth #3
    triggerInfo.toothAngles[3] = 480; //tooth #4
  }
  else
  {
    //Should be 4 cylinders here
    triggerInfo.toothAngles[0] = 0; //tooth #1
    triggerInfo.toothAngles[1] = 30; //tooth #2 (Extra tooth)
    triggerInfo.toothAngles[2] = 180; //tooth #3
    triggerInfo.toothAngles[3] = 360; //tooth #4
    triggerInfo.toothAngles[4] = 540; //tooth #5
  }
}

void triggerPri_Daihatsu(void)
{
  triggerInfo.curTime = micros();
  triggerInfo.curGap = triggerInfo.curTime - triggerInfo.toothLastToothTime;

  //if ( triggerInfo.curGap >= triggerInfo.triggerFilterTime || (currentStatus.startRevolutions == 0 )
  {
    triggerInfo.toothSystemCount++;
    BIT_SET(triggerInfo.decoderState, BIT_DECODER_VALID_TRIGGER); //Flag this pulse as being a valid trigger (ie that it passed filters)

    if (currentStatus.hasSync == true)
    {
      if( (triggerInfo.toothCurrentCount == triggerInfo.triggerActualTeeth) ) //Check if we're back to the beginning of a revolution
      {
         triggerInfo.toothCurrentCount = 1; //Reset the counter
         triggerInfo.toothOneMinusOneTime = triggerInfo.toothOneTime;
         triggerInfo.toothOneTime = triggerInfo.curTime;
         currentStatus.hasSync = true;
         currentStatus.startRevolutions++; //Counter

         //Need to set a special filter time for the next tooth
         triggerInfo.triggerFilterTime = 20; //Fix this later
      }
      else
      {
        triggerInfo.toothCurrentCount++; //Increment the tooth counter
        setFilter(triggerInfo.curGap); //Recalc the new filter value
      }

      if ( configPage4.ignCranklock && BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) )
      {
        //This locks the cranking timing to 0 degrees BTDC (All the triggers allow for)
        if(triggerInfo.toothCurrentCount == 1) { endCoil1Charge(); }
        else if(triggerInfo.toothCurrentCount == 2) { endCoil2Charge(); }
        else if(triggerInfo.toothCurrentCount == 3) { endCoil3Charge(); }
        else if(triggerInfo.toothCurrentCount == 4) { endCoil4Charge(); }
      }
    }
    else //NO SYNC
    {
      //
      if(triggerInfo.toothSystemCount >= 3) //Need to have seen at least 3 teeth to determine SYNC
      {
        unsigned long targetTime;
        //We need to try and find the extra tooth (#2) which is located 30 degrees after tooth #1
        //Aim for tooth times less than about 60 degrees
        if(configPage2.nCylinders == 3)
        {
          targetTime = (triggerInfo.toothLastToothTime -  triggerInfo.toothLastMinusOneToothTime) / 4; //Teeth are 240 degrees apart for 3 cylinder. 240/4 = 60
        }
        else
        {
          targetTime = ((triggerInfo.toothLastToothTime -  triggerInfo.toothLastMinusOneToothTime) * 3) / 8; //Teeth are 180 degrees apart for 4 cylinder. (180*3)/8 = 67
        }
        if(triggerInfo.curGap < targetTime)
        {
          //Means we're on the extra tooth here
          triggerInfo.toothCurrentCount = 2; //Reset the counter
          currentStatus.hasSync = true;
          triggerInfo.triggerFilterTime = targetTime; //Lazy, but it works
        }
      }
    }

    triggerInfo.toothLastMinusOneToothTime = triggerInfo.toothLastToothTime;
    triggerInfo.toothLastToothTime = triggerInfo.curTime;
  } //Trigger filter
}
void triggerSec_Daihatsu(void) { return; } //Not required (Should never be called in the first place)

uint16_t getRPM_Daihatsu(void)
{
  uint16_t tempRPM = 0;
  if( (currentStatus.RPM < currentStatus.crankRPM) && false) //Disable special cranking processing for now
  {
    //Can't use standard cranking RPM function due to extra tooth
    if( currentStatus.hasSync == true )
    {
      if(triggerInfo.toothCurrentCount == 2) { tempRPM = currentStatus.RPM; }
      else if (triggerInfo.toothCurrentCount == 3) { tempRPM = currentStatus.RPM; }
      else
      {
        noInterrupts();
        SetRevolutionTime((triggerInfo.toothLastToothTime - triggerInfo.toothLastMinusOneToothTime) * (triggerInfo.triggerActualTeeth-1));
        interrupts();
        tempRPM = RpmFromRevolutionTimeUs(revolutionTime);
      } //is tooth #2
    }
    else { tempRPM = 0; } //No sync
  }
  else
  { tempRPM = stdGetRPM(CAM_SPEED); } //Tracking over 2 crank revolutions

  return tempRPM;

}
int getCrankAngle_Daihatsu(void)
{
    //This is the current angle ATDC the engine is at. This is the last known position based on what tooth was last 'seen'. It is only accurate to the resolution of the trigger wheel (Eg 36-1 is 10 degrees)
    unsigned long temptoothLastToothTime;
    int temptoothCurrentCount;
    int crankAngle;
    //Grab some variables that are used in the trigger code and assign them to temp variables.
    noInterrupts();
    temptoothCurrentCount = triggerInfo.toothCurrentCount;
    temptoothLastToothTime = triggerInfo.toothLastToothTime;
    triggerInfo.lastCrankAngleCalc = micros(); //micros() is no longer interrupt safe
    interrupts();

    crankAngle = triggerInfo.toothAngles[temptoothCurrentCount-1] + configPage4.triggerAngle; //Crank angle of the last tooth seen

    //Estimate the number of degrees travelled since the last tooth}
    triggerInfo.elapsedTime = (triggerInfo.lastCrankAngleCalc - temptoothLastToothTime);
    crankAngle += timeToAngleDegPerMicroSec(triggerInfo.elapsedTime);

    if (crankAngle >= 720) { crankAngle -= 720; }
    if (crankAngle < 0) { crankAngle += CRANK_ANGLE_MAX; }

    return crankAngle;
}

void triggerSetEndTeeth_Daihatsu(void)
{
}
/** @} */

