
/** Honda D17 (1.7 liter 4 cyl SOHC).
*
* @defgroup dec_honda_d17 Honda D17
* @{
*/
void triggerSetup_HondaD17(void)
{
  triggerInfo.triggerToothAngle = 360 / 12; //The number of degrees that passes from tooth to tooth
  MAX_STALL_TIME = ((MICROS_PER_DEG_1_RPM/50U) * triggerInfo.triggerToothAngle); //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
  BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_2ND_DERIV);
  BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_IS_SEQUENTIAL);
  BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_HAS_SECONDARY);
}

void triggerPri_HondaD17(void)
{
   triggerInfo.lastGap = triggerInfo.curGap;
   triggerInfo.curTime = micros();
   triggerInfo.curGap = triggerInfo.curTime - triggerInfo.toothLastToothTime;
   triggerInfo.toothCurrentCount++; //Increment the tooth counter

   BIT_SET(triggerInfo.decoderState, BIT_DECODER_VALID_TRIGGER); //Flag this pulse as being a valid trigger (ie that it passed filters)

   //
   if( (triggerInfo.toothCurrentCount == 13) && (currentStatus.hasSync == true) )
   {
     triggerInfo.toothCurrentCount = 0;
   }
   else if( (triggerInfo.toothCurrentCount == 1) && (currentStatus.hasSync == true) )
   {
     triggerInfo.toothOneMinusOneTime = triggerInfo.toothOneTime;
     triggerInfo.toothOneTime = triggerInfo.curTime;
     currentStatus.startRevolutions++; //Counter

     triggerInfo.toothLastMinusOneToothTime = triggerInfo.toothLastToothTime;
     triggerInfo.toothLastToothTime = triggerInfo.curTime;
   }
   else
   {
     //13th tooth
     triggerInfo.targetGap = (triggerInfo.lastGap) >> 1; //The target gap is set at half the last tooth gap
     if ( triggerInfo.curGap < triggerInfo.targetGap) //If the gap between this tooth and the last one is less than half of the previous gap, then we are very likely at the magical 13th tooth
     {
       triggerInfo.toothCurrentCount = 0;
       currentStatus.hasSync = true;
     }
     else
     {
       //The tooth times below don't get set on tooth 13(The magical 13th tooth should not be considered for any calculations that use those times)
       triggerInfo.toothLastMinusOneToothTime = triggerInfo.toothLastToothTime;
       triggerInfo.toothLastToothTime = triggerInfo.curTime;
     }
   }

}
void triggerSec_HondaD17(void) { return; } //The 4+1 signal on the cam is yet to be supported. If this ever changes, update BIT_DECODER_HAS_SECONDARY in the setup() function
uint16_t getRPM_HondaD17(void)
{
   return stdGetRPM(CRANK_SPEED);
}
int getCrankAngle_HondaD17(void)
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

    //Check if the last tooth seen was the reference tooth 13 (Number 0 here). All others can be calculated, but tooth 3 has a unique angle
    int crankAngle;
    if( temptoothCurrentCount == 0 )
    {
      crankAngle = (11 * triggerInfo.triggerToothAngle) + configPage4.triggerAngle; //if temptoothCurrentCount is 0, the last tooth seen was the 13th one. Based on this, ignore the 13th tooth and use the 12th one as the last reference.
    }
    else
    {
      crankAngle = ((temptoothCurrentCount - 1) * triggerInfo.triggerToothAngle) + configPage4.triggerAngle; //Number of teeth that have passed since tooth 1, multiplied by the angle each tooth represents, plus the angle that tooth 1 is ATDC. This gives accuracy only to the nearest tooth.
    }

    //Estimate the number of degrees travelled since the last tooth}
    triggerInfo.elapsedTime = (triggerInfo.lastCrankAngleCalc - temptoothLastToothTime);
    crankAngle += timeToAngleDegPerMicroSec(triggerInfo.elapsedTime);

    if (crankAngle >= 720) { crankAngle -= 720; }
    if (crankAngle < 0) { crankAngle += 360; }

    return crankAngle;
}

void triggerSetEndTeeth_HondaD17(void)
{
}


