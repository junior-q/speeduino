

/** Subaru 6/7 Trigger pattern decoder for 6 tooth (irregularly spaced) crank and 7 tooth (also fairly irregular) cam wheels (eg late 90's Impreza 2.2).
This seems to be present in late 90's Subaru. In 2001 Subaru moved to 36-2-2-2 (See: http://www.vems.hu/wiki/index.php?page=InputTrigger%2FSubaruTrigger ).
* @defgroup dec_subaru_6_7 Subaru 6/7
* @{
*/
void triggerSetup_Subaru67(void)
{
  triggerInfo.triggerFilterTime = (MICROS_PER_SEC / (MAX_RPM / 60U * 360UL)); //Trigger filter time is the shortest possible time (in uS) that there can be between crank teeth (ie at max RPM). Any pulses that occur faster than this time will be discarded as noise
  triggerInfo.triggerSecFilterTime = 0;
  triggerInfo.secondaryToothCount = 0; //Initially set to 0 prior to calculating the secondary window duration
  BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_2ND_DERIV);
  BIT_SET(triggerInfo.decoderState, BIT_DECODER_IS_SEQUENTIAL);
  BIT_SET(triggerInfo.decoderState, BIT_DECODER_HAS_SECONDARY);
  triggerInfo.toothCurrentCount = 1;
  triggerInfo.triggerToothAngle = 2;
  BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);
  triggerInfo.toothSystemCount = 0;
  MAX_STALL_TIME = ((MICROS_PER_DEG_1_RPM/50U) * 93U); //Minimum 50rpm. (3333uS is the time per degree at 50rpm)

  triggerInfo.toothAngles[0] = 710; //tooth #1
  triggerInfo.toothAngles[1] = 83; //tooth #2
  triggerInfo.toothAngles[2] = 115; //tooth #3
  triggerInfo.toothAngles[3] = 170; //tooth #4
  triggerInfo.toothAngles[4] = triggerInfo.toothAngles[1] + 180;
  triggerInfo.toothAngles[5] = triggerInfo.toothAngles[2] + 180;
  triggerInfo.toothAngles[6] = triggerInfo.toothAngles[3] + 180;
  triggerInfo.toothAngles[7] = triggerInfo.toothAngles[1] + 360;
  triggerInfo.toothAngles[8] = triggerInfo.toothAngles[2] + 360;
  triggerInfo.toothAngles[9] = triggerInfo.toothAngles[3] + 360;
  triggerInfo.toothAngles[10] = triggerInfo.toothAngles[1] + 540;
  triggerInfo.toothAngles[11] = triggerInfo.toothAngles[2] + 540;
}


void triggerPri_Subaru67(void)
{
  triggerInfo.curTime = micros();
  triggerInfo.curGap = triggerInfo.curTime - triggerInfo.toothLastToothTime;
  if ( triggerInfo.curGap < triggerInfo.triggerFilterTime )
  { return; }

  triggerInfo.toothCurrentCount++; //Increment the tooth counter
  triggerInfo.toothSystemCount++; //Used to count the number of primary pulses that have occurred since the last secondary. Is part of the noise filtering system.
  BIT_SET(triggerInfo.decoderState, BIT_DECODER_VALID_TRIGGER); //Flag this pulse as being a valid trigger (ie that it passed filters)

  triggerInfo.toothLastMinusOneToothTime = triggerInfo.toothLastToothTime;
  triggerInfo.toothLastToothTime = triggerInfo.curTime;


  if(triggerInfo.toothCurrentCount > 13) //can't have more than 12 teeth so have lost sync
  {
    triggerInfo.toothCurrentCount = 0;
    currentStatus.hasSync = false;
    currentStatus.syncLossCounter++;
  }

  //Sync is determined by counting the number of cam teeth that have passed between the crank teeth
  switch(triggerInfo.secondaryToothCount)
  {
    case 0:
      //If no teeth have passed, we can't do anything
      break;

    case 1:
      //Can't do anything with a single pulse from the cam either (We need either 2 or 3 pulses)
      if(triggerInfo.toothCurrentCount == 5 || triggerInfo.toothCurrentCount == 11)
      { currentStatus.hasSync = true; }
      else
      {
        currentStatus.hasSync = false;
        currentStatus.syncLossCounter++;
        triggerInfo.toothCurrentCount = 5; // we don't know if its 5 or 11, but we'll be right 50% of the time and speed up getting sync 50%
      }
      triggerInfo.secondaryToothCount = 0;
      break;

    case 2:
      if (triggerInfo.toothCurrentCount == 8)
      {  currentStatus.hasSync = true; }
      else
      {
        currentStatus.hasSync = false;
        currentStatus.syncLossCounter++;
        triggerInfo.toothCurrentCount = 8;
      }
      triggerInfo.secondaryToothCount = 0;
      break;

    case 3:
      if( triggerInfo.toothCurrentCount == 2)
      {  currentStatus.hasSync = true; }
      else
      {
        currentStatus.hasSync = false;
        currentStatus.syncLossCounter++;
        triggerInfo.toothCurrentCount = 2;
      }
      triggerInfo.secondaryToothCount = 0;
      break;

    default:
      //Almost certainly due to noise or cranking stop/start
      currentStatus.hasSync = false;
      BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);
      currentStatus.syncLossCounter++;
      triggerInfo.secondaryToothCount = 0;
      break;
  }

  //Check sync again
  if ( currentStatus.hasSync == true )
  {
    //Locked timing during cranking. This is fixed at 10* BTDC.
    if ( BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) && configPage4.ignCranklock)
    {
      if( (triggerInfo.toothCurrentCount == 1) || (triggerInfo.toothCurrentCount == 7) ) { endCoil1Charge(); endCoil3Charge(); }
      else if( (triggerInfo.toothCurrentCount == 4) || (triggerInfo.toothCurrentCount == 10) ) { endCoil2Charge(); endCoil4Charge(); }
    }

    if ( triggerInfo.toothCurrentCount > 12 ) // done 720 degrees so increment rotation
    {
      triggerInfo.toothCurrentCount = 1;
      triggerInfo.toothOneMinusOneTime = triggerInfo.toothOneTime;
      triggerInfo.toothOneTime = triggerInfo.curTime;
      currentStatus.startRevolutions++; //Counter
    }

    //Set the last angle between teeth for better calc accuracy
    if(triggerInfo.toothCurrentCount == 1) { triggerInfo.triggerToothAngle = 55; } //Special case for tooth 1
    else if(triggerInfo.toothCurrentCount == 2) { triggerInfo.triggerToothAngle = 93; } //Special case for tooth 2
    else { triggerInfo.triggerToothAngle = triggerInfo.toothAngles[(triggerInfo.toothCurrentCount-1)] - triggerInfo.toothAngles[(triggerInfo.toothCurrentCount-2)]; }
    BIT_SET(triggerInfo.decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);


    //NEW IGNITION MODE
    if( (configPage2.perToothIgn == true) && (!BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK)) )
    {
      int16_t crankAngle = triggerInfo.toothAngles[(triggerInfo.toothCurrentCount - 1)] + configPage4.triggerAngle;
      if( (configPage4.sparkMode != IGN_MODE_SEQUENTIAL) )
      {
        crankAngle = ignitionLimits( triggerInfo.toothAngles[(triggerInfo.toothCurrentCount-1)] );

        //Handle non-sequential tooth counts
        if( (configPage4.sparkMode != IGN_MODE_SEQUENTIAL) && (triggerInfo.toothCurrentCount > 6) ) { checkPerToothTiming(crankAngle, (triggerInfo.toothCurrentCount-6) ); }
        else { checkPerToothTiming(crankAngle, triggerInfo.toothCurrentCount); }
      }
      else{ checkPerToothTiming(crankAngle, triggerInfo.toothCurrentCount); }
    }
  //Recalc the new filter value
  //setFilter(triggerInfo.curGap);
  }
 }

void triggerSec_Subaru67(void)
{
  if( ((triggerInfo.toothSystemCount == 0) || (triggerInfo.toothSystemCount == 3)) )
  {
    triggerInfo.curTime2 = micros();
    triggerInfo.curGap2 = triggerInfo.curTime2 - triggerInfo.toothLastSecToothTime;

    if ( triggerInfo.curGap2 > triggerInfo.triggerSecFilterTime )
    {
      triggerInfo.toothLastSecToothTime = triggerInfo.curTime2;
      triggerInfo.secondaryToothCount++;
      triggerInfo.toothSystemCount = 0;

      if(triggerInfo.secondaryToothCount > 1)
      {
        //Set filter at 25% of the current speed
        //Note that this can only be set on the 2nd or 3rd cam tooth in each set.
        triggerInfo.triggerSecFilterTime = triggerInfo.curGap2 >> 2;
      }
      else { triggerInfo.triggerSecFilterTime = 0; } //Filter disabled

    }
  }
  else
  {
    //Sanity check
    if(triggerInfo.toothSystemCount > 3)
    {
      triggerInfo.toothSystemCount = 0;
      triggerInfo.secondaryToothCount = 1;
      currentStatus.hasSync = false; // impossible to have more than 3 crank teeth between cam teeth - must have noise but can't have sync
      currentStatus.syncLossCounter++;
    }
    triggerInfo.secondaryToothCount = 0;
  }

}

uint16_t getRPM_Subaru67(void)
{
  //if(currentStatus.RPM < currentStatus.crankRPM) { return crankingGetRPM(configPage4.triggerTeeth); }

  uint16_t tempRPM = 0;
  if(currentStatus.startRevolutions > 0)
  {
    //As the tooth count is over 720 degrees
    tempRPM = stdGetRPM(CAM_SPEED);
  }
  return tempRPM;
}

int getCrankAngle_Subaru67(void)
{
  int crankAngle = 0;
  if( currentStatus.hasSync == true )
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
    crankAngle += timeToAngleIntervalTooth(triggerInfo.elapsedTime);

    if (crankAngle >= 720) { crankAngle -= 720; }
    if (crankAngle < 0) { crankAngle += 360; }
  }

  return crankAngle;
}

void triggerSetEndTeeth_Subaru67(void)
{
  if(configPage4.sparkMode == IGN_MODE_SEQUENTIAL)
  {
    //if(ignition1EndAngle < 710) { triggerInfo.ignition1EndTooth = 12; }
    if(currentStatus.advance >= 10 )
    {
      triggerInfo.ignition1EndTooth = 12;
      triggerInfo.ignition2EndTooth = 3;
      triggerInfo.ignition3EndTooth = 6;
      triggerInfo.ignition4EndTooth = 9;
    }
    else
    {
      triggerInfo.ignition1EndTooth = 1;
      triggerInfo.ignition2EndTooth = 4;
      triggerInfo.ignition3EndTooth = 7;
      triggerInfo.ignition4EndTooth = 10;
    }
  }
  else
  {
    if(currentStatus.advance >= 10 )
    {
      triggerInfo.ignition1EndTooth = 6;
      triggerInfo.ignition2EndTooth = 3;
      //triggerInfo.ignition3EndTooth = 6;
      //triggerInfo.ignition4EndTooth = 9;
    }
    else
    {
      triggerInfo.ignition1EndTooth = 1;
      triggerInfo.ignition2EndTooth = 4;
      //triggerInfo.ignition3EndTooth = 7;
      //triggerInfo.ignition4EndTooth = 10;
    }
  }
}
/** @} */


