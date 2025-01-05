
/** DSM 420a, For the DSM Eclipse with 16 teeth total on the crank.
* Tracks the falling side of the signal.
* Sync is determined by watching for a falling edge on the secondary signal and checking if the primary signal is high then.
* https://github.com/noisymime/speeduino/issues/133
* @defgroup dec_dsm_420a DSM 420a, For the DSM Eclipse
* @{
*/
void triggerSetup_420a(void)
{
  triggerInfo.triggerFilterTime = (MICROS_PER_SEC / (MAX_RPM / 60U * 360UL)); //Trigger filter time is the shortest possible time (in uS) that there can be between crank teeth (ie at max RPM). Any pulses that occur faster than this time will be discarded as noise
  triggerInfo.triggerSecFilterTime = 0;
  triggerInfo.secondaryToothCount = 0; //Initially set to 0 prior to calculating the secondary window duration
  BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_2ND_DERIV);
  BIT_SET(triggerInfo.decoderState, BIT_DECODER_IS_SEQUENTIAL);
  BIT_SET(triggerInfo.decoderState, BIT_DECODER_HAS_SECONDARY);
  triggerInfo.toothCurrentCount = 1;
  triggerInfo.triggerToothAngle = 20; //Is only correct for the 4 short pulses before each TDC
  BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);
  triggerInfo.toothSystemCount = 0;
  MAX_STALL_TIME = ((MICROS_PER_DEG_1_RPM/50U) * 93U); //Minimum 50rpm. (3333uS is the time per degree at 50rpm)

  triggerInfo.toothAngles[0] = 711; //tooth #1, just before #1 TDC
  triggerInfo.toothAngles[1] = 111;
  triggerInfo.toothAngles[2] = 131;
  triggerInfo.toothAngles[3] = 151;
  triggerInfo.toothAngles[4] = 171; //Just before #3 TDC
  triggerInfo.toothAngles[5] = triggerInfo.toothAngles[1] + 180;
  triggerInfo.toothAngles[6] = triggerInfo.toothAngles[2] + 180;
  triggerInfo.toothAngles[7] = triggerInfo.toothAngles[3] + 180;
  triggerInfo.toothAngles[8] = triggerInfo.toothAngles[4] + 180; //Just before #4 TDC
  triggerInfo.toothAngles[9]  = triggerInfo.toothAngles[1] + 360;
  triggerInfo.toothAngles[10] = triggerInfo.toothAngles[2] + 360;
  triggerInfo.toothAngles[11] = triggerInfo.toothAngles[3] + 360;
  triggerInfo.toothAngles[12] = triggerInfo.toothAngles[4] + 360; //Just before #2 TDC
  triggerInfo.toothAngles[13] = triggerInfo.toothAngles[1] + 540;
  triggerInfo.toothAngles[14] = triggerInfo.toothAngles[2] + 540;
  triggerInfo.toothAngles[15] = triggerInfo.toothAngles[3] + 540;
}

void triggerPri_420a(void)
{
  triggerInfo.curTime = micros();
  triggerInfo.curGap = triggerInfo.curTime - triggerInfo.toothLastToothTime;
  if ( triggerInfo.curGap >= triggerInfo.triggerFilterTime ) //Pulses should never be less than triggerInfo.triggerFilterTime, so if they are it means a false trigger. (A 36-1 wheel at 8000pm will have triggers approx. every 200uS)
  {
    triggerInfo.toothCurrentCount++; //Increment the tooth counter
    BIT_SET(triggerInfo.decoderState, BIT_DECODER_VALID_TRIGGER); //Flag this pulse as being a valid trigger (ie that it passed filters)

    if( (triggerInfo.toothLastToothTime == 0) || (triggerInfo.toothLastMinusOneToothTime == 0) ) { triggerInfo.curGap = 0; }

    if( (triggerInfo.toothCurrentCount > 16) && (currentStatus.hasSync == true) )
    {
      //Means a complete rotation has occurred.
      triggerInfo.toothCurrentCount = 1;
      triggerInfo.toothOneMinusOneTime = triggerInfo.toothOneTime;
      triggerInfo.toothOneTime = triggerInfo.curTime;
      currentStatus.startRevolutions++; //Counter
    }

    //Filter can only be recalculated for the regular teeth, not the missing one.
    //setFilter(triggerInfo.curGap);
    triggerInfo.triggerFilterTime = 0;

    BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);

    triggerInfo.toothLastMinusOneToothTime = triggerInfo.toothLastToothTime;
    triggerInfo.toothLastToothTime = triggerInfo.curTime;

    //EXPERIMENTAL!
    if(configPage2.perToothIgn == true)
    {
      int16_t crankAngle = ( triggerInfo.toothAngles[(triggerInfo.toothCurrentCount-1)] ) + configPage4.triggerAngle;
      crankAngle = ignitionLimits(crankAngle);
      checkPerToothTiming(crankAngle, triggerInfo.toothCurrentCount);
    }
  }
}

void triggerSec_420a(void)
{
  //Secondary trigger is only on falling edge

  if(READ_PRI_TRIGGER() == true)
  {
    //Secondary signal is falling and primary signal is HIGH
    if( currentStatus.hasSync == false )
    {
      //If we don't have sync, then assume the signal is good
      triggerInfo.toothCurrentCount = 13;
      currentStatus.hasSync = true;
    }
    else
    {
      //If we DO have sync, then check that the tooth count matches what we expect
      if(triggerInfo.toothCurrentCount != 13)
      {
        currentStatus.syncLossCounter++;
        triggerInfo.toothCurrentCount = 13;
      }
    }

  }
  else
  {
    //Secondary signal is falling and primary signal is LOW
    if( currentStatus.hasSync == false )
    {
      //If we don't have sync, then assume the signal is good
      triggerInfo.toothCurrentCount = 5;
      currentStatus.hasSync = true;
    }
    else
    {
      //If we DO have sync, then check that the tooth count matches what we expect
      if(triggerInfo.toothCurrentCount != 5)
      {
        currentStatus.syncLossCounter++;
        triggerInfo.toothCurrentCount = 5;
      }
    }
  }
}

uint16_t getRPM_420a(void)
{
  uint16_t tempRPM = 0;
  if( currentStatus.RPM < currentStatus.crankRPM)
  {
    //Possibly look at doing special handling for cranking in the future, but for now just use the standard method
    tempRPM = stdGetRPM(CAM_SPEED);
  }
  else
  {
    tempRPM = stdGetRPM(CAM_SPEED);
  }
  return tempRPM;
}

int getCrankAngle_420a(void)
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
  crankAngle = triggerInfo.toothAngles[(temptoothCurrentCount - 1)] + configPage4.triggerAngle; //Perform a lookup of the fixed triggerInfo.toothAngles array to find what the angle of the last tooth passed was.

  //Estimate the number of degrees travelled since the last tooth}
  triggerInfo.elapsedTime = (triggerInfo.lastCrankAngleCalc - temptoothLastToothTime);
  crankAngle += timeToAngleDegPerMicroSec(triggerInfo.elapsedTime);

  if (crankAngle >= 720) { crankAngle -= 720; }
  if (crankAngle < 0) { crankAngle += 360; }

  return crankAngle;
}

void triggerSetEndTeeth_420a(void)
{
  if(currentStatus.advance < 9)
  {
    triggerInfo.ignition1EndTooth = 1;
    triggerInfo.ignition2EndTooth = 5;
    triggerInfo.ignition3EndTooth = 9;
    triggerInfo.ignition4EndTooth = 13;
  }
  else
  {
    triggerInfo.ignition1EndTooth = 16;
    triggerInfo.ignition2EndTooth = 4;
    triggerInfo.ignition3EndTooth = 8;
    triggerInfo.ignition4EndTooth = 12;
  }
}
/** @} */

