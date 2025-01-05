

/** Decode GM 7X trigger wheel with six equally spaced teeth and a seventh tooth for cylinder identification.
* Note: Within the decoder code pf GM7X, the sync tooth is referred to as tooth #3 rather than tooth #7. This makes for simpler angle calculations
* (See: http://www.speeduino.com/forum/download/file.php?id=4743 ).
* @defgroup dec_gm7x GM7X
* @{
*/
void triggerSetup_GM7X(void)
{
  triggerInfo.triggerToothAngle = 360 / 6; //The number of degrees that passes from tooth to tooth
  BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_2ND_DERIV);
  BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_IS_SEQUENTIAL);
  BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_HAS_SECONDARY);
  MAX_STALL_TIME = ((MICROS_PER_DEG_1_RPM/50U) * triggerInfo.triggerToothAngle); //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
}

void triggerPri_GM7X(void)
{
    triggerInfo.lastGap = triggerInfo.curGap;
    triggerInfo.curTime = micros();
    triggerInfo.curGap = triggerInfo.curTime - triggerInfo.toothLastToothTime;
    triggerInfo.toothCurrentCount++; //Increment the tooth counter
    BIT_SET(triggerInfo.decoderState, BIT_DECODER_VALID_TRIGGER); //Flag this pulse as being a valid trigger (ie that it passed filters)

    if( (triggerInfo.toothLastToothTime > 0) && (triggerInfo.toothLastMinusOneToothTime > 0) )
    {
      if( triggerInfo.toothCurrentCount > 7 )
      {
        triggerInfo.toothCurrentCount = 1;
        triggerInfo.toothOneMinusOneTime = triggerInfo.toothOneTime;
        triggerInfo.toothOneTime = triggerInfo.curTime;

        BIT_SET(triggerInfo.decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);
      }
      else
      {
        triggerInfo.targetGap = (triggerInfo.lastGap) >> 1; //The target gap is set at half the last tooth gap
        if ( triggerInfo.curGap < triggerInfo.targetGap ) //If the gap between this tooth and the last one is less than half of the previous gap, then we are very likely at the magical 3rd tooth
        {
          triggerInfo.toothCurrentCount = 3;
          currentStatus.hasSync = true;
          BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_TOOTH_ANG_CORRECT); //The tooth angle is double at this point
          currentStatus.startRevolutions++; //Counter
        }
        else
        {
          BIT_SET(triggerInfo.decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);
        }
      }
    }

    //New ignition mode!
    if(configPage2.perToothIgn == true)
    {
      if(triggerInfo.toothCurrentCount != 3) //Never do the check on the extra tooth. It's not needed anyway
      {
        //configPage4.triggerAngle must currently be below 48 and above -81
        int16_t crankAngle;
        if( triggerInfo.toothCurrentCount < 3 )
        {
          crankAngle = ((triggerInfo.toothCurrentCount - 1) * triggerInfo.triggerToothAngle) + 42 + configPage4.triggerAngle; //Number of teeth that have passed since tooth 1, multiplied by the angle each tooth represents, plus the angle that tooth 1 is ATDC. This gives accuracy only to the nearest tooth.
        }
        else
        {
          crankAngle = ((triggerInfo.toothCurrentCount - 2) * triggerInfo.triggerToothAngle) + 42 + configPage4.triggerAngle; //Number of teeth that have passed since tooth 1, multiplied by the angle each tooth represents, plus the angle that tooth 1 is ATDC. This gives accuracy only to the nearest tooth.
        }
        checkPerToothTiming(crankAngle, triggerInfo.toothCurrentCount);
      }
    }

    triggerInfo.toothLastMinusOneToothTime = triggerInfo.toothLastToothTime;
    triggerInfo.toothLastToothTime = triggerInfo.curTime;


}
void triggerSec_GM7X(void) { return; } //Not required
uint16_t getRPM_GM7X(void)
{
   return stdGetRPM(CRANK_SPEED);
}
int getCrankAngle_GM7X(void)
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
    if( temptoothCurrentCount < 3 )
    {
      crankAngle = ((temptoothCurrentCount - 1) * triggerInfo.triggerToothAngle) + 42 + configPage4.triggerAngle; //Number of teeth that have passed since tooth 1, multiplied by the angle each tooth represents, plus the angle that tooth 1 is ATDC. This gives accuracy only to the nearest tooth.
    }
    else if( temptoothCurrentCount == 3 )
    {
      crankAngle = 112;
    }
    else
    {
      crankAngle = ((temptoothCurrentCount - 2) * triggerInfo.triggerToothAngle) + 42 + configPage4.triggerAngle; //Number of teeth that have passed since tooth 1, multiplied by the angle each tooth represents, plus the angle that tooth 1 is ATDC. This gives accuracy only to the nearest tooth.
    }

    //Estimate the number of degrees travelled since the last tooth}
    triggerInfo.elapsedTime = (triggerInfo.lastCrankAngleCalc - temptoothLastToothTime);
    crankAngle += timeToAngleDegPerMicroSec(triggerInfo.elapsedTime);

    if (crankAngle >= 720) { crankAngle -= 720; }
    if (crankAngle < 0) { crankAngle += 360; }

    return crankAngle;
}

void triggerSetEndTeeth_GM7X(void)
{
  if(currentStatus.advance < 18 )
  {
    triggerInfo.ignition1EndTooth = 7;
    triggerInfo.ignition2EndTooth = 2;
    triggerInfo.ignition3EndTooth = 5;
  }
  else
  {
    triggerInfo.ignition1EndTooth = 6;
    triggerInfo.ignition2EndTooth = 1;
    triggerInfo.ignition3EndTooth = 4;
  }
}
/** @} */

