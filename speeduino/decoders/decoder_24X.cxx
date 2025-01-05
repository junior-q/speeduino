

/** GM 24X Decoder (eg early LS1 1996-2005).
Note: Useful references:
*
- www.vems.hu/wiki/index.php?page=MembersPage%2FJorgenKarlsson%2FTwentyFourX

Provided that the cam signal is used, this decoder simply counts the teeth and then looks their angles up against a lookup table. The cam signal is used to determine tooth #1
* @defgroup dec_gm GM 24X
* @{
*/
void triggerSetup_24X(void)
{
  triggerInfo.triggerToothAngle = 15; //The number of degrees that passes from tooth to tooth (primary)
  triggerInfo.toothAngles[0] = 12;
  triggerInfo.toothAngles[1] = 18;
  triggerInfo.toothAngles[2] = 33;
  triggerInfo.toothAngles[3] = 48;
  triggerInfo.toothAngles[4] = 63;
  triggerInfo.toothAngles[5] = 78;
  triggerInfo.toothAngles[6] = 102;
  triggerInfo.toothAngles[7] = 108;
  triggerInfo.toothAngles[8] = 123;
  triggerInfo.toothAngles[9] = 138;
  triggerInfo.toothAngles[10] = 162;
  triggerInfo.toothAngles[11] = 177;
  triggerInfo.toothAngles[12] = 183;
  triggerInfo.toothAngles[13] = 198;
  triggerInfo.toothAngles[14] = 222;
  triggerInfo.toothAngles[15] = 237;
  triggerInfo.toothAngles[16] = 252;
  triggerInfo.toothAngles[17] = 258;
  triggerInfo.toothAngles[18] = 282;
  triggerInfo.toothAngles[19] = 288;
  triggerInfo.toothAngles[20] = 312;
  triggerInfo.toothAngles[21] = 327;
  triggerInfo.toothAngles[22] = 342;
  triggerInfo.toothAngles[23] = 357;

  MAX_STALL_TIME = ((MICROS_PER_DEG_1_RPM/50U) * triggerInfo.triggerToothAngle); //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
  if(currentStatus.initialisationComplete == false) { triggerInfo.toothCurrentCount = 25; triggerInfo.toothLastToothTime = micros(); } //Set a startup value here to avoid filter errors when starting. This MUST have the init check to prevent the fuel pump just staying on all the time
  BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_2ND_DERIV);
  BIT_SET(triggerInfo.decoderState, BIT_DECODER_IS_SEQUENTIAL);
  BIT_SET(triggerInfo.decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);
  BIT_SET(triggerInfo.decoderState, BIT_DECODER_HAS_SECONDARY);
}

void triggerPri_24X(void)
{
  if(triggerInfo.toothCurrentCount == 25) { currentStatus.hasSync = false; } //Indicates sync has not been achieved (Still waiting for 1 revolution of the crank to take place)
  else
  {
    triggerInfo.curTime = micros();
    triggerInfo.curGap = triggerInfo.curTime - triggerInfo.toothLastToothTime;

    if(triggerInfo.toothCurrentCount == 0)
    {
       triggerInfo.toothCurrentCount = 1; //Reset the counter
       triggerInfo.toothOneMinusOneTime = triggerInfo.toothOneTime;
       triggerInfo.toothOneTime = triggerInfo.curTime;
       triggerInfo.revolutionOne = !triggerInfo.revolutionOne; //Sequential revolution flip
       currentStatus.hasSync = true;
       currentStatus.startRevolutions++; //Counter
       triggerInfo.triggerToothAngle = 15; //Always 15 degrees for tooth #15
    }
    else
    {
      triggerInfo.toothCurrentCount++; //Increment the tooth counter
      triggerInfo.triggerToothAngle = triggerInfo.toothAngles[(triggerInfo.toothCurrentCount-1)] - triggerInfo.toothAngles[(triggerInfo.toothCurrentCount-2)]; //Calculate the last tooth gap in degrees
    }

    BIT_SET(triggerInfo.decoderState, BIT_DECODER_VALID_TRIGGER); //Flag this pulse as being a valid trigger (ie that it passed filters)

    triggerInfo.toothLastToothTime = triggerInfo.curTime;


  }
}

void triggerSec_24X(void)
{
  triggerInfo.toothCurrentCount = 0; //All we need to do is reset the tooth count back to zero, indicating that we're at the beginning of a new revolution
  triggerInfo.revolutionOne = 1; //Sequential revolution reset
}

uint16_t getRPM_24X(void)
{
   return stdGetRPM(CRANK_SPEED);
}
int getCrankAngle_24X(void)
{
    //This is the current angle ATDC the engine is at. This is the last known position based on what tooth was last 'seen'. It is only accurate to the resolution of the trigger wheel (Eg 36-1 is 10 degrees)
    unsigned long temptoothLastToothTime;
    int temptoothCurrentCount, temprevolutionOne;
    //Grab some variables that are used in the trigger code and assign them to temp variables.
    noInterrupts();
    temptoothCurrentCount = triggerInfo.toothCurrentCount;
    temptoothLastToothTime = triggerInfo.toothLastToothTime;
    temprevolutionOne = triggerInfo.revolutionOne;
    triggerInfo.lastCrankAngleCalc = micros(); //micros() is no longer interrupt safe
    interrupts();

    int crankAngle;
    if (temptoothCurrentCount == 0) { crankAngle = 0 + configPage4.triggerAngle; } //This is the special case to handle when the 'last tooth' seen was the cam tooth. 0 is the angle at which the crank tooth goes high (Within 360 degrees).
    else { crankAngle = triggerInfo.toothAngles[(temptoothCurrentCount - 1)] + configPage4.triggerAngle;} //Perform a lookup of the fixed triggerInfo.toothAngles array to find what the angle of the last tooth passed was.

    //Estimate the number of degrees travelled since the last tooth}
    triggerInfo.elapsedTime = (triggerInfo.lastCrankAngleCalc - temptoothLastToothTime);
    crankAngle += timeToAngleDegPerMicroSec(triggerInfo.elapsedTime);

    //Sequential check (simply sets whether we're on the first or 2nd revolution of the cycle)
    if (temprevolutionOne == 1) { crankAngle += 360; }

    if (crankAngle >= 720) { crankAngle -= 720; }
    if (crankAngle < 0) { crankAngle += 360; }

    return crankAngle;
}

void triggerSetEndTeeth_24X(void)
{
}
/** @} */

