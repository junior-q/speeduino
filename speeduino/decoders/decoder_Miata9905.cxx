

/** Miata '99 to '05 with 4x 70 degree duration teeth running at cam speed.
Teeth believed to be at the same angles as the 4g63 decoder.
Tooth #1 is defined as the next crank tooth after the crank signal is HIGH when the cam signal is falling.
Tooth number one is at 355* ATDC.
* (See: www.forum.diyefi.org/viewtopic.php?f=56&t=1077)
* @defgroup miata_99_05 Miata '99 to '05
* @{
*/
void triggerSetup_Miata9905(void)
{
  triggerInfo.triggerToothAngle = 90; //The number of degrees that passes from tooth to tooth (primary)
  triggerInfo.toothCurrentCount = 99; //Fake tooth count represents no sync
  BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_2ND_DERIV);
  BIT_SET(triggerInfo.decoderState, BIT_DECODER_IS_SEQUENTIAL);
  triggerInfo.triggerActualTeeth = 8;

  if(currentStatus.initialisationComplete == false) { triggerInfo.secondaryToothCount = 0; triggerInfo.toothLastToothTime = micros(); } //Set a startup value here to avoid filter errors when starting. This MUST have the initial check to prevent the fuel pump just staying on all the time
  else { triggerInfo.toothLastToothTime = 0; }
  triggerInfo.toothLastMinusOneToothTime = 0;

  //Note that these angles are for every rising and falling edge

  /*
  triggerInfo.toothAngles[0] = 350;
  triggerInfo.toothAngles[1] = 100;
  triggerInfo.toothAngles[2] = 170;
  triggerInfo.toothAngles[3] = 280;
  */

  triggerInfo.toothAngles[0] = 710; //
  triggerInfo.toothAngles[1] = 100; //First crank pulse after the SINGLE cam pulse
  triggerInfo.toothAngles[2] = 170; //
  triggerInfo.toothAngles[3] = 280; //
  triggerInfo.toothAngles[4] = 350; //
  triggerInfo.toothAngles[5] = 460; //First crank pulse AFTER the DOUBLE cam pulse
  triggerInfo.toothAngles[6] = 530; //
  triggerInfo.toothAngles[7] = 640; //

  MAX_STALL_TIME = ((MICROS_PER_DEG_1_RPM/50U) * triggerInfo.triggerToothAngle); //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
  triggerInfo.triggerFilterTime = 1500; //10000 rpm, assuming we're triggering on both edges off the crank tooth.
  triggerInfo.triggerSecFilterTime = 0; //Need to figure out something better for this
  BIT_SET(triggerInfo.decoderState, BIT_DECODER_HAS_FIXED_CRANKING);
  BIT_SET(triggerInfo.decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);
  BIT_SET(triggerInfo.decoderState, BIT_DECODER_HAS_SECONDARY);
}

void triggerPri_Miata9905(void)
{
  triggerInfo.curTime = micros();
  triggerInfo.curGap = triggerInfo.curTime - triggerInfo.toothLastToothTime;
  if ( (triggerInfo.curGap >= triggerInfo.triggerFilterTime) || (currentStatus.startRevolutions == 0) )
  {
    triggerInfo.toothCurrentCount++;
    BIT_SET(triggerInfo.decoderState, BIT_DECODER_VALID_TRIGGER); //Flag this pulse as being a valid trigger (ie that it passed filters)
    if( (triggerInfo.toothCurrentCount == (triggerInfo.triggerActualTeeth + 1)) )
    {
       triggerInfo.toothCurrentCount = 1; //Reset the counter
       triggerInfo.toothOneMinusOneTime = triggerInfo.toothOneTime;
       triggerInfo.toothOneTime = triggerInfo.curTime;
       //currentStatus.hasSync = true;
       currentStatus.startRevolutions++; //Counter
    }
    else
    {
      if( (currentStatus.hasSync == false) || (configPage4.useResync == true) )
      {
        if(triggerInfo.secondaryToothCount == 2)
        {
          triggerInfo.toothCurrentCount = 6;
          currentStatus.hasSync = true;
        }
      }
    }

    if (currentStatus.hasSync == true)
    {

      //Whilst this is an uneven tooth pattern, if the specific angle between the last 2 teeth is specified, 1st deriv prediction can be used
      if( (configPage4.triggerFilter == 1) || (currentStatus.RPM < 1400) )
      {
        //Lite filter
        if( (triggerInfo.toothCurrentCount == 1) || (triggerInfo.toothCurrentCount == 3) || (triggerInfo.toothCurrentCount == 5) || (triggerInfo.toothCurrentCount == 7) ) { triggerInfo.triggerToothAngle = 70; triggerInfo.triggerFilterTime = triggerInfo.curGap; } //Trigger filter is set to whatever time it took to do 70 degrees (Next trigger is 110 degrees away)
        else { triggerInfo.triggerToothAngle = 110; triggerInfo.triggerFilterTime = rshift<3>(triggerInfo.curGap * 3UL); } //Trigger filter is set to (110*3)/8=41.25=41 degrees (Next trigger is 70 degrees away).
      }
      else if(configPage4.triggerFilter == 2)
      {
        //Medium filter level
        if( (triggerInfo.toothCurrentCount == 1) || (triggerInfo.toothCurrentCount == 3) || (triggerInfo.toothCurrentCount == 5) || (triggerInfo.toothCurrentCount == 7) ) { triggerInfo.triggerToothAngle = 70; triggerInfo.triggerFilterTime = (triggerInfo.curGap * 5) >> 2 ; } //87.5 degrees with a target of 110
        else { triggerInfo.triggerToothAngle = 110; triggerInfo.triggerFilterTime = (triggerInfo.curGap >> 1); } //55 degrees with a target of 70
      }
      else if (configPage4.triggerFilter == 3)
      {
        //Aggressive filter level
        if( (triggerInfo.toothCurrentCount == 1) || (triggerInfo.toothCurrentCount == 3) || (triggerInfo.toothCurrentCount == 5) || (triggerInfo.toothCurrentCount == 7) ) { triggerInfo.triggerToothAngle = 70; triggerInfo.triggerFilterTime = rshift<3>(triggerInfo.curGap * 11UL) ; } //96.26 degrees with a target of 110
        else { triggerInfo.triggerToothAngle = 110; triggerInfo.triggerFilterTime = rshift<5>(triggerInfo.curGap * 9UL); } //61.87 degrees with a target of 70
      }
      else if (configPage4.triggerFilter == 0)
      {
        //trigger filter is turned off.
        triggerInfo.triggerFilterTime = 0;
        triggerInfo.triggerSecFilterTime = 0;
        if( (triggerInfo.toothCurrentCount == 1) || (triggerInfo.toothCurrentCount == 3) || (triggerInfo.toothCurrentCount == 5) || (triggerInfo.toothCurrentCount == 7) ) { triggerInfo.triggerToothAngle = 70; } //96.26 degrees with a target of 110
        else { triggerInfo.triggerToothAngle = 110; }
      }

      //EXPERIMENTAL!
      //New ignition mode is ONLY available on 9905 when the trigger angle is set to the stock value of 0.
      if(    (configPage2.perToothIgn == true)
          && (configPage4.triggerAngle == 0)
          && (currentStatus.advance > 0) )
      {
        int16_t crankAngle = ignitionLimits( triggerInfo.toothAngles[(triggerInfo.toothCurrentCount-1)] );

        //Handle non-sequential tooth counts
        if( (configPage4.sparkMode != IGN_MODE_SEQUENTIAL) && (triggerInfo.toothCurrentCount > configPage2.nCylinders) ) { checkPerToothTiming(crankAngle, (triggerInfo.toothCurrentCount-configPage2.nCylinders) ); }
        else { checkPerToothTiming(crankAngle, triggerInfo.toothCurrentCount); }
      }
    } //Has sync

    triggerInfo.toothLastMinusOneToothTime = triggerInfo.toothLastToothTime;
    triggerInfo.toothLastToothTime = triggerInfo.curTime;

    //if ( BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) && configPage4.ignCranklock)
    if ( (currentStatus.RPM < (currentStatus.crankRPM + 30)) && (configPage4.ignCranklock) ) //The +30 here is a safety margin. When switching from fixed timing to normal, there can be a situation where a pulse started when fixed and ending when in normal mode causes problems. This prevents that.
    {
      if( (triggerInfo.toothCurrentCount == 1) || (triggerInfo.toothCurrentCount == 5) ) { endCoil1Charge(); endCoil3Charge(); }
      else if( (triggerInfo.toothCurrentCount == 3) || (triggerInfo.toothCurrentCount == 7) ) { endCoil2Charge(); endCoil4Charge(); }
    }
    triggerInfo.secondaryToothCount = 0;
  } //Trigger filter

}

void triggerSec_Miata9905(void)
{
  triggerInfo.curTime2 = micros();
  triggerInfo.curGap2 = triggerInfo.curTime2 - triggerInfo.toothLastSecToothTime;

  if(BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) || (currentStatus.hasSync == false) )
  {
    triggerInfo.triggerFilterTime = 1500; //If this is removed, can have trouble getting sync again after the engine is turned off (but ECU not reset).
  }

  if ( triggerInfo.curGap2 >= triggerInfo.triggerSecFilterTime )
  {
    triggerInfo.toothLastSecToothTime = triggerInfo.curTime2;
    triggerInfo.lastGap = triggerInfo.curGap2;
    triggerInfo.secondaryToothCount++;

    //TODO Add some secondary filtering here

    //Record the VVT tooth time
    if( (triggerInfo.toothCurrentCount == 1) && (triggerInfo.curTime2 > triggerInfo.toothLastToothTime) )
    {
      triggerInfo.lastVVTtime = triggerInfo.curTime2 - triggerInfo.toothLastToothTime;
    }
  }
}

uint16_t getRPM_Miata9905(void)
{
  //During cranking, RPM is calculated 4 times per revolution, once for each tooth on the crank signal.
  //Because these signals aren't even (Alternating 110 and 70 degrees), this needs a special function
  uint16_t tempRPM = 0;
  if( (currentStatus.RPM < currentStatus.crankRPM) && (currentStatus.hasSync == true) )
  {
    if( (triggerInfo.toothLastToothTime == 0) || (triggerInfo.toothLastMinusOneToothTime == 0) ) { tempRPM = 0; }
    else
    {
      int tempToothAngle;
      unsigned long toothTime;
      noInterrupts();
      tempToothAngle = triggerInfo.triggerToothAngle;
      toothTime = (triggerInfo.toothLastToothTime - triggerInfo.toothLastMinusOneToothTime); //Note that trigger tooth angle changes between 70 and 110 depending on the last tooth that was seen
      interrupts();
      toothTime = toothTime * 36;
      tempRPM = ((unsigned long)tempToothAngle * (MICROS_PER_MIN/10U)) / toothTime;
      SetRevolutionTime((10UL * toothTime) / tempToothAngle);
      MAX_STALL_TIME = 366667UL; // 50RPM
    }
  }
  else
  {
    tempRPM = stdGetRPM(CAM_SPEED);
    MAX_STALL_TIME = revolutionTime << 1; //Set the stall time to be twice the current RPM. This is a safe figure as there should be no single revolution where this changes more than this
    if(MAX_STALL_TIME < 366667UL) { MAX_STALL_TIME = 366667UL; } //Check for 50rpm minimum
  }

  return tempRPM;
}

int getCrankAngle_Miata9905(void)
{
    int crankAngle = 0;
    //if(currentStatus.hasSync == true)
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

int getCamAngle_Miata9905(void)
{
  int16_t curAngle;
  //triggerInfo.lastVVTtime is the time between tooth #1 (10* BTDC) and the single cam tooth.
  //All cam angles in in BTDC, so the actual advance angle is 370 - timeToAngleDegPerMicroSec(triggerInfo.lastVVTtime) - <the angle of the cam at 0 advance>
  curAngle = 370 - timeToAngleDegPerMicroSec(triggerInfo.lastVVTtime) - configPage10.vvtCL0DutyAng;
  currentStatus.vvt1Angle = LOW_PASS_FILTER( (curAngle << 1), configPage4.ANGLEFILTER_VVT, currentStatus.vvt1Angle);

  return currentStatus.vvt1Angle;
}

void triggerSetEndTeeth_Miata9905(void)
{

  if(configPage4.sparkMode == IGN_MODE_SEQUENTIAL)
  {
    if(currentStatus.advance >= 10)
    {
      triggerInfo.ignition1EndTooth = 8;
      triggerInfo.ignition2EndTooth = 2;
      triggerInfo.ignition3EndTooth = 4;
      triggerInfo.ignition4EndTooth = 6;
    }
    else if (currentStatus.advance > 0)
    {
      triggerInfo.ignition1EndTooth = 1;
      triggerInfo.ignition2EndTooth = 3;
      triggerInfo.ignition3EndTooth = 5;
      triggerInfo.ignition4EndTooth = 7;
    }

  }
  else
  {
    if(currentStatus.advance >= 10)
    {
      triggerInfo.ignition1EndTooth = 4;
      triggerInfo.ignition2EndTooth = 2;
      triggerInfo.ignition3EndTooth = 4; //Not used
      triggerInfo.ignition4EndTooth = 2; //Not used
    }
    else if(currentStatus.advance > 0)
    {
      triggerInfo.ignition1EndTooth = 1;
      triggerInfo.ignition2EndTooth = 3;
      triggerInfo.ignition3EndTooth = 1; //Not used
      triggerInfo.ignition4EndTooth = 3; //Not used
    }
  }
}
/** @} */

