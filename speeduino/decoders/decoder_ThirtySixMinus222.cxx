
//************************************************************************************************************************

/** 36-2-2-2 crank based trigger wheel.
* A crank based trigger with a nominal 36 teeth, but 6 of these removed in 3 groups of 2.
* 2 of these groups are located concurrently.
* Note: This decoder supports both the H4 version (13-missing-16-missing-1-missing) and the H6 version of 36-2-2-2 (19-missing-10-missing-1-missing).
* The decoder checks which pattern is selected in order to determine the tooth number
* Note: www.thefactoryfiveforum.com/attachment.php?attachmentid=34279&d=1412431418
*
* @defgroup dec_36_2_2_2 36-2-2-2 Trigger wheel
* @{
*/
void triggerSetup_ThirtySixMinus222(void)
{
  triggerInfo.triggerToothAngle = 10; //The number of degrees that passes from tooth to tooth
  triggerInfo.triggerActualTeeth = 30; //The number of physical teeth on the wheel. Doing this here saves us a calculation each time in the interrupt
  triggerInfo.triggerFilterTime = (int)(MICROS_PER_SEC / (MAX_RPM / 60U * 36)); //Trigger filter time is the shortest possible time (in uS) that there can be between crank teeth (ie at max RPM). Any pulses that occur faster than this time will be discarded as noise
  BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_2ND_DERIV);
  BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_IS_SEQUENTIAL);
  BIT_SET(triggerInfo.decoderState, BIT_DECODER_HAS_SECONDARY);
  triggerInfo.checkSyncToothCount = (configPage4.triggerTeeth) >> 1; //50% of the total teeth.
  triggerInfo.toothLastMinusOneToothTime = 0;
  triggerInfo.toothCurrentCount = 0;
  triggerInfo.toothOneTime = 0;
  triggerInfo.toothOneMinusOneTime = 0;
  MAX_STALL_TIME = ((MICROS_PER_DEG_1_RPM/50U) * triggerInfo.triggerToothAngle * 2U ); //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
}

void triggerPri_ThirtySixMinus222(void)
{
   triggerInfo.curTime = micros();
   triggerInfo.curGap = triggerInfo.curTime - triggerInfo.toothLastToothTime;
   if ( triggerInfo.curGap >= triggerInfo.triggerFilterTime ) //Pulses should never be less than triggerInfo.triggerFilterTime, so if they are it means a false trigger. (A 36-1 wheel at 8000pm will have triggers approx. every 200uS)
   {
     triggerInfo.toothCurrentCount++; //Increment the tooth counter
     BIT_SET(triggerInfo.decoderState, BIT_DECODER_VALID_TRIGGER); //Flag this pulse as being a valid trigger (ie that it passed filters)

     //Begin the missing tooth detection
     //If the time between the current tooth and the last is greater than 2x the time between the last tooth and the tooth before that, we make the assertion that we must be at the first tooth after a gap
     //triggerInfo.toothSystemCount is used to keep track of which missed tooth we're on. It will be set to 1 if that last tooth seen was the middle one in the -2-2 area. At all other times it will be 0
     if(triggerInfo.toothSystemCount == 0) { triggerInfo.targetGap = ((triggerInfo.toothLastToothTime - triggerInfo.toothLastMinusOneToothTime)) * 2; } //Multiply by 2 (Checks for a gap 2x greater than the last one)


     if( (triggerInfo.toothLastToothTime == 0) || (triggerInfo.toothLastMinusOneToothTime == 0) ) { triggerInfo.curGap = 0; }

     if ( (triggerInfo.curGap > triggerInfo.targetGap) )
     {
       {
         if(triggerInfo.toothSystemCount == 1)
         {
           //This occurs when we're at the first tooth after the 2 lots of 2x missing tooth.
           if(configPage2.nCylinders == 4 ) { triggerInfo.toothCurrentCount = 19; } //H4
           else if(configPage2.nCylinders == 6) { triggerInfo.toothCurrentCount = 12; } //H6 - NOT TESTED!

           triggerInfo.toothSystemCount = 0;
           currentStatus.hasSync = true;
         }
         else
         {
           //We've seen a missing tooth set, but do not yet know whether it is the single one or the double one.
           triggerInfo.toothSystemCount = 1;
           triggerInfo.toothCurrentCount++;
           triggerInfo.toothCurrentCount++; //Accurately reflect the actual tooth count, including the skipped ones
         }
         BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_TOOTH_ANG_CORRECT); //The tooth angle is double at this point
         triggerInfo.triggerFilterTime = 0; //This is used to prevent a condition where serious intermittent signals (Eg someone furiously plugging the sensor wire in and out) can leave the filter in an unrecoverable state
       }
     }
     else
     {
       if(triggerInfo.toothCurrentCount > 36)
       {
         //Means a complete rotation has occurred.
         triggerInfo.toothCurrentCount = 1;
         triggerInfo.revolutionOne = !triggerInfo.revolutionOne; //Flip sequential revolution tracker
         triggerInfo.toothOneMinusOneTime = triggerInfo.toothOneTime;
         triggerInfo.toothOneTime = triggerInfo.curTime;
         currentStatus.startRevolutions++; //Counter

       }
       else if(triggerInfo.toothSystemCount == 1)
       {
          //This occurs when a set of missing teeth had been seen, but the next one was NOT missing.
          if(configPage2.nCylinders == 4 )
          {
            //H4
            triggerInfo.toothCurrentCount = 35;
            currentStatus.hasSync = true;
          }
          else if(configPage2.nCylinders == 6)
          {
            //H6 - THIS NEEDS TESTING
            triggerInfo.toothCurrentCount = 34;
            currentStatus.hasSync = true;
          }

       }

       //Filter can only be recalculated for the regular teeth, not the missing one.
       setFilter(triggerInfo.curGap);

       BIT_SET(triggerInfo.decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);
       triggerInfo.toothSystemCount = 0;
     }

     triggerInfo.toothLastMinusOneToothTime = triggerInfo.toothLastToothTime;
     triggerInfo.toothLastToothTime = triggerInfo.curTime;

     //EXPERIMENTAL!
     if(configPage2.perToothIgn == true)
     {
       int16_t crankAngle = ( (triggerInfo.toothCurrentCount-1) * triggerInfo.triggerToothAngle ) + configPage4.triggerAngle;
       crankAngle = ignitionLimits(crankAngle);
       checkPerToothTiming(crankAngle, triggerInfo.toothCurrentCount);
     }

   }
}

void triggerSec_ThirtySixMinus222(void)
{
  //NOT USED - This pattern uses the missing tooth version of this function
}

uint16_t getRPM_ThirtySixMinus222(void)
{
  uint16_t tempRPM = 0;
  if( currentStatus.RPM < currentStatus.crankRPM)
  {

    if( (configPage2.nCylinders == 4) && (triggerInfo.toothCurrentCount != 19) && (triggerInfo.toothCurrentCount != 16) && (triggerInfo.toothCurrentCount != 34) && (BIT_CHECK(triggerInfo.decoderState, BIT_DECODER_TOOTH_ANG_CORRECT)) )
    {
      tempRPM = crankingGetRPM(36, CRANK_SPEED);
    }
    else if( (configPage2.nCylinders == 6) && (triggerInfo.toothCurrentCount != 9) && (triggerInfo.toothCurrentCount != 12) && (triggerInfo.toothCurrentCount != 33) && (BIT_CHECK(triggerInfo.decoderState, BIT_DECODER_TOOTH_ANG_CORRECT)) )
    {
      tempRPM = crankingGetRPM(36, CRANK_SPEED);
    }
    else { tempRPM = currentStatus.RPM; } //Can't do per tooth RPM if we're at and of the missing teeth as it messes the calculation
  }
  else
  {
    tempRPM = stdGetRPM(CRANK_SPEED);
  }
  return tempRPM;
}

int getCrankAngle_ThirtySixMinus222(void)
{
    //NOT USED - This pattern uses the missing tooth version of this function
    return 0;
}

void triggerSetEndTeeth_ThirtySixMinus222(void)
{
  if(configPage2.nCylinders == 4 )
  {
    if(currentStatus.advance < 10) { triggerInfo.ignition1EndTooth = 36; }
    else if(currentStatus.advance < 20) { triggerInfo.ignition1EndTooth = 35; }
    else if(currentStatus.advance < 30) { triggerInfo.ignition1EndTooth = 34; }
    else { triggerInfo.ignition1EndTooth = 31; }

    if(currentStatus.advance < 30) { triggerInfo.ignition2EndTooth = 16; }
    else { triggerInfo.ignition2EndTooth = 13; }
  }
  else if(configPage2.nCylinders == 6)
  {
    //H6
    if(currentStatus.advance < 10) { triggerInfo.ignition1EndTooth = 36; }
    else if(currentStatus.advance < 20) { triggerInfo.ignition1EndTooth = 35; }
    else if(currentStatus.advance < 30) { triggerInfo.ignition1EndTooth = 34; }
    else if(currentStatus.advance < 40) { triggerInfo.ignition1EndTooth = 33; }
    else { triggerInfo.ignition1EndTooth = 31; }

    if(currentStatus.advance < 20) { triggerInfo.ignition2EndTooth = 9; }
    else { triggerInfo.ignition2EndTooth = 6; }

    if(currentStatus.advance < 10) { triggerInfo.ignition3EndTooth = 23; }
    else if(currentStatus.advance < 20) { triggerInfo.ignition3EndTooth = 22; }
    else if(currentStatus.advance < 30) { triggerInfo.ignition3EndTooth = 21; }
    else if(currentStatus.advance < 40) { triggerInfo.ignition3EndTooth = 20; }
    else { triggerInfo.ignition3EndTooth = 19; }
  }
}
/** @} */

