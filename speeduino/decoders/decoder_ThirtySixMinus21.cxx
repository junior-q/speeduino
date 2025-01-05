


/** 36-2-1 / Mistsubishi 4B11 - A crank based trigger with a nominal 36 teeth, but with 1 single and 1 double missing tooth.
* @defgroup dec_36_2_1 36-2-1 For Mistsubishi 4B11
* @{
*/
void triggerSetup_ThirtySixMinus21(void)
{
  triggerInfo.triggerToothAngle = 10; //The number of degrees that passes from tooth to tooth
  triggerInfo.triggerActualTeeth = 33; //The number of physical teeth on the wheel. Doing this here saves us a calculation each time in the interrupt. Not Used
  triggerInfo.triggerFilterTime = (MICROS_PER_SEC / (MAX_RPM / 60U * 36)); //Trigger filter time is the shortest possible time (in uS) that there can be between crank teeth (ie at max RPM). Any pulses that occur faster than this time will be discarded as noise
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

void triggerPri_ThirtySixMinus21(void)
{
   triggerInfo.curTime = micros();
   triggerInfo.curGap = triggerInfo.curTime - triggerInfo.toothLastToothTime;
   if ( triggerInfo.curGap >= triggerInfo.triggerFilterTime ) //Pulses should never be less than triggerInfo.triggerFilterTime, so if they are it means a false trigger. (A 36-1 wheel at 8000pm will have triggers approx. every 200uS)
   {
     triggerInfo.toothCurrentCount++; //Increment the tooth counter
     BIT_SET(triggerInfo.decoderState, BIT_DECODER_VALID_TRIGGER); //Flag this pulse as being a valid trigger (ie that it passed filters)

     //Begin the missing tooth detection
     //If the time between the current tooth and the last is greater than 2x the time between the last tooth and the tooth before that, we make the assertion that we must be at the first tooth after a gap

     triggerInfo.targetGap2 = (3 * (triggerInfo.toothLastToothTime - triggerInfo.toothLastMinusOneToothTime)) ; //Multiply by 3 (Checks for a gap 3x greater than the last one)
     triggerInfo.targetGap = triggerInfo.targetGap2 >> 1;  //Multiply by 1.5 (Checks for a gap 1.5x greater than the last one) (Uses bitshift to divide by 2 as in the missing tooth decoder)

     if( (triggerInfo.toothLastToothTime == 0) || (triggerInfo.toothLastMinusOneToothTime == 0) ) { triggerInfo.curGap = 0; }

     if ( (triggerInfo.curGap > triggerInfo.targetGap) )
     {
      if ( (triggerInfo.curGap < triggerInfo.targetGap2))
       {
           //we are at the tooth after the single gap
           triggerInfo.toothCurrentCount = 20; //it's either 19 or 20, need to clarify engine direction!
           currentStatus.hasSync = true;
        }
        else
        {
          //we are at the tooth after the double gap
          triggerInfo.toothCurrentCount = 1;
          currentStatus.hasSync = true;
        }

         BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_TOOTH_ANG_CORRECT); //The tooth angle is double at this point
         triggerInfo.triggerFilterTime = 0; //This is used to prevent a condition where serious intermittent signals (Eg someone furiously plugging the sensor wire in and out) can leave the filter in an unrecoverable state
       }
     }
     else
     {
       if(  (triggerInfo.toothCurrentCount > 36) || ( triggerInfo.toothCurrentCount==1)  )
       {
         //Means a complete rotation has occurred.
         triggerInfo.toothCurrentCount = 1;
         triggerInfo.revolutionOne = !triggerInfo.revolutionOne; //Flip sequential revolution tracker
         triggerInfo.toothOneMinusOneTime = triggerInfo.toothOneTime;
         triggerInfo.toothOneTime = triggerInfo.curTime;
         currentStatus.startRevolutions++; //Counter

       }

       //Filter can only be recalculated for the regular teeth, not the missing one.
       setFilter(triggerInfo.curGap);

       BIT_SET(triggerInfo.decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);

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

void triggerSec_ThirtySixMinus21(void)
{
  //NOT USED - This pattern uses the missing tooth version of this function
}

uint16_t getRPM_ThirtySixMinus21(void)
{
  uint16_t tempRPM = 0;
  if( currentStatus.RPM < currentStatus.crankRPM)
  {
    if( (triggerInfo.toothCurrentCount != 20) && (BIT_CHECK(triggerInfo.decoderState, BIT_DECODER_TOOTH_ANG_CORRECT)) )
    {
      tempRPM = crankingGetRPM(36, CRANK_SPEED);
    }
    else { tempRPM = currentStatus.RPM; } //Can't do per tooth RPM if we're at tooth #1 as the missing tooth messes the calculation
  }
  else
  {
    tempRPM = stdGetRPM(CRANK_SPEED);
  }
  return tempRPM;
}

int getCrankAngle_ThirtySixMinus21(void)
{
    //NOT USED - This pattern uses the missing tooth version of this function
    return 0;
}

void triggerSetEndTeeth_ThirtySixMinus21(void)
{
  triggerInfo.ignition1EndTooth = 10;
  triggerInfo.ignition2EndTooth = 28; // Arbitrarily picked  at 180°.
}
/** @} */

