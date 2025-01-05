

/** Mitsubishi 4G63 / NA/NB Miata + MX-5 / 4/2.
Note: raw.githubusercontent.com/noisymime/speeduino/master/reference/wiki/decoders/4g63_trace.png
Tooth #1 is defined as the next crank tooth after the crank signal is HIGH when the cam signal is falling.
Tooth number one is at 355* ATDC.
* @defgroup dec_mitsu_miata Mistsubishi 4G63 and Miata + MX-5
* @{
*/
void triggerSetup_4G63(void)
{
  triggerInfo.triggerToothAngle = 180; //The number of degrees that passes from tooth to tooth (primary)
  triggerInfo.toothCurrentCount = 99; //Fake tooth count represents no sync
  BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_2ND_DERIV);
  BIT_SET(triggerInfo.decoderState, BIT_DECODER_IS_SEQUENTIAL);
  BIT_SET(triggerInfo.decoderState, BIT_DECODER_HAS_FIXED_CRANKING);
  BIT_SET(triggerInfo.decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);
  BIT_SET(triggerInfo.decoderState, BIT_DECODER_HAS_SECONDARY);
  MAX_STALL_TIME = 366667UL; //Minimum 50rpm based on the 110 degree tooth spacing
  if(currentStatus.initialisationComplete == false) { triggerInfo.toothLastToothTime = micros(); } //Set a startup value here to avoid filter errors when starting. This MUST have the initial check to prevent the fuel pump just staying on all the time

  //Note that these angles are for every rising and falling edge
  if(configPage2.nCylinders == 6)
  {
    //New values below
    triggerInfo.toothAngles[0] = 715; //Rising edge of tooth #1
    triggerInfo.toothAngles[1] = 45;  //Falling edge of tooth #1
    triggerInfo.toothAngles[2] = 115; //Rising edge of tooth #2
    triggerInfo.toothAngles[3] = 165; //Falling edge of tooth #2
    triggerInfo.toothAngles[4] = 235; //Rising edge of tooth #3
    triggerInfo.toothAngles[5] = 285; //Falling edge of tooth #3

    triggerInfo.toothAngles[6] = 355; //Rising edge of tooth #4
    triggerInfo.toothAngles[7] = 405; //Falling edge of tooth #4
    triggerInfo.toothAngles[8] = 475; //Rising edge of tooth #5
    triggerInfo.toothAngles[9] = 525; //Falling edge of tooth $5
    triggerInfo.toothAngles[10] = 595; //Rising edge of tooth #6
    triggerInfo.toothAngles[11] = 645; //Falling edge of tooth #6

    triggerInfo.triggerActualTeeth = 12; //Both sides of all teeth over 720 degrees
  }
  else
  {
    // 70 / 110 for 4 cylinder
    triggerInfo.toothAngles[0] = 715; //Falling edge of tooth #1
    triggerInfo.toothAngles[1] = 105; //Rising edge of tooth #2
    triggerInfo.toothAngles[2] = 175; //Falling edge of tooth #2
    triggerInfo.toothAngles[3] = 285; //Rising edge of tooth #1

    triggerInfo.toothAngles[4] = 355; //Falling edge of tooth #1
    triggerInfo.toothAngles[5] = 465; //Rising edge of tooth #2
    triggerInfo.toothAngles[6] = 535; //Falling edge of tooth #2
    triggerInfo.toothAngles[7] = 645; //Rising edge of tooth #1

    triggerInfo.triggerActualTeeth = 8;
  }

  triggerInfo.triggerFilterTime = 1500; //10000 rpm, assuming we're triggering on both edges off the crank tooth.
  triggerInfo.triggerSecFilterTime = (int)(MICROS_PER_SEC / (MAX_RPM / 60U * 2U)) / 2U; //Same as above, but fixed at 2 teeth on the secondary input and divided by 2 (for cam speed)
  triggerInfo.triggerSecFilterTime_duration = 4000;
  triggerInfo.secondaryLastToothTime = 0;
}

void triggerPri_4G63(void)
{
  triggerInfo.curTime = micros();
  triggerInfo.curGap = triggerInfo.curTime - triggerInfo.toothLastToothTime;
  if ( (triggerInfo.curGap >= triggerInfo.triggerFilterTime) || (currentStatus.startRevolutions == 0) )
  {
    BIT_SET(triggerInfo.decoderState, BIT_DECODER_VALID_TRIGGER); //Flag this pulse as being a valid trigger (ie that it passed filters)
    triggerInfo.triggerFilterTime = triggerInfo.curGap >> 2; //This only applies during non-sync conditions. If there is sync then triggerInfo.triggerFilterTime gets changed again below with a better value.

    triggerInfo.toothLastMinusOneToothTime = triggerInfo.toothLastToothTime;
    triggerInfo.toothLastToothTime = triggerInfo.curTime;

    triggerInfo.toothCurrentCount++;

    if( (triggerInfo.toothCurrentCount == 1) || (triggerInfo.toothCurrentCount > triggerInfo.triggerActualTeeth) ) //Trigger is on CHANGE, hence 4 pulses = 1 crank rev (or 6 pulses for 6 cylinders)
    {
       triggerInfo.toothCurrentCount = 1; //Reset the counter
       triggerInfo.toothOneMinusOneTime = triggerInfo.toothOneTime;
       triggerInfo.toothOneTime = triggerInfo.curTime;
       currentStatus.startRevolutions++; //Counter
    }

    if (currentStatus.hasSync == true)
    {
      if ( BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) && configPage4.ignCranklock && (currentStatus.startRevolutions >= configPage4.StgCycles))
      {
        if(configPage2.nCylinders == 4)
        {
          //This operates in forced wasted spark mode during cranking to align with crank teeth
          if( (triggerInfo.toothCurrentCount == 1) || (triggerInfo.toothCurrentCount == 5) ) { endCoil1Charge(); endCoil3Charge(); }
          else if( (triggerInfo.toothCurrentCount == 3) || (triggerInfo.toothCurrentCount == 7) ) { endCoil2Charge(); endCoil4Charge(); }
        }
        else if(configPage2.nCylinders == 6)
        {
          if( (triggerInfo.toothCurrentCount == 1) || (triggerInfo.toothCurrentCount == 7) ) { endCoil1Charge(); }
          else if( (triggerInfo.toothCurrentCount == 3) || (triggerInfo.toothCurrentCount == 9) ) { endCoil2Charge(); }
          else if( (triggerInfo.toothCurrentCount == 5) || (triggerInfo.toothCurrentCount == 11) ) { endCoil3Charge(); }
        }
      }

      //Whilst this is an uneven tooth pattern, if the specific angle between the last 2 teeth is specified, 1st deriv prediction can be used
      if( (configPage4.triggerFilter == 1) || (currentStatus.RPM < 1400) )
      {
        //Lite filter
        if( (triggerInfo.toothCurrentCount == 1) || (triggerInfo.toothCurrentCount == 3) || (triggerInfo.toothCurrentCount == 5) || (triggerInfo.toothCurrentCount == 7) || (triggerInfo.toothCurrentCount == 9) || (triggerInfo.toothCurrentCount == 11) )
        {
          if(configPage2.nCylinders == 4)
          {
            triggerInfo.triggerToothAngle = 70;
            triggerInfo.triggerFilterTime = triggerInfo.curGap; //Trigger filter is set to whatever time it took to do 70 degrees (Next trigger is 110 degrees away)
          }
          else if(configPage2.nCylinders == 6)
          {
            triggerInfo.triggerToothAngle = 70;
            triggerInfo.triggerFilterTime = (triggerInfo.curGap >> 2); //Trigger filter is set to (70/4)=17.5=17 degrees (Next trigger is 50 degrees away).
          }
        }
        else
        {
          if(configPage2.nCylinders == 4)
          {
            triggerInfo.triggerToothAngle = 110;
            triggerInfo.triggerFilterTime = rshift<3>(triggerInfo.curGap * 3UL); //Trigger filter is set to (110*3)/8=41.25=41 degrees (Next trigger is 70 degrees away).
          }
          else if(configPage2.nCylinders == 6)
          {
            triggerInfo.triggerToothAngle = 50;
            triggerInfo.triggerFilterTime = triggerInfo.curGap >> 1; //Trigger filter is set to 25 degrees (Next trigger is 70 degrees away).
          }
        }
      }
      else if(configPage4.triggerFilter == 2)
      {
        //Medium filter level
        if( (triggerInfo.toothCurrentCount == 1) || (triggerInfo.toothCurrentCount == 3) || (triggerInfo.toothCurrentCount == 5) || (triggerInfo.toothCurrentCount == 7) || (triggerInfo.toothCurrentCount == 9) || (triggerInfo.toothCurrentCount == 11) )
        {
          triggerInfo.triggerToothAngle = 70;
          if(configPage2.nCylinders == 4)
          {
            triggerInfo.triggerFilterTime = (triggerInfo.curGap * 5) >> 2 ; //87.5 degrees with a target of 110
          }
          else
          {
            triggerInfo.triggerFilterTime = triggerInfo.curGap >> 1 ; //35 degrees with a target of 50
          }
        }
        else
        {
          if(configPage2.nCylinders == 4)
          {
            triggerInfo.triggerToothAngle = 110;
            triggerInfo.triggerFilterTime = (triggerInfo.curGap >> 1); //55 degrees with a target of 70
          }
          else
          {
            triggerInfo.triggerToothAngle = 50;
            triggerInfo.triggerFilterTime = (triggerInfo.curGap * 3) >> 2; //Trigger filter is set to (50*3)/4=37.5=37 degrees (Next trigger is 70 degrees away).
          }
        }
      }
      else if (configPage4.triggerFilter == 3)
      {
        //Aggressive filter level
        if( (triggerInfo.toothCurrentCount == 1) || (triggerInfo.toothCurrentCount == 3) || (triggerInfo.toothCurrentCount == 5) || (triggerInfo.toothCurrentCount == 7) || (triggerInfo.toothCurrentCount == 9) || (triggerInfo.toothCurrentCount == 11) )
        {
          triggerInfo.triggerToothAngle = 70;
          if(configPage2.nCylinders == 4)
          {
            triggerInfo.triggerFilterTime = rshift<3>(triggerInfo.curGap * 11UL);//96.26 degrees with a target of 110
          }
          else
          {
            triggerInfo.triggerFilterTime = triggerInfo.curGap >> 1 ; //35 degrees with a target of 50
          }
        }
        else
        {
          if(configPage2.nCylinders == 4)
          {
            triggerInfo.triggerToothAngle = 110;
            triggerInfo.triggerFilterTime = rshift<5>(triggerInfo.curGap * 9UL); //61.87 degrees with a target of 70
          }
          else
          {
            triggerInfo.triggerToothAngle = 50;
            triggerInfo.triggerFilterTime = triggerInfo.curGap; //50 degrees with a target of 70
          }
        }
      }
      else
      {
        //trigger filter is turned off.
        triggerInfo.triggerFilterTime = 0;
        if( (triggerInfo.toothCurrentCount == 1) || (triggerInfo.toothCurrentCount == 3) || (triggerInfo.toothCurrentCount == 5) || (triggerInfo.toothCurrentCount == 7) || (triggerInfo.toothCurrentCount == 9) || (triggerInfo.toothCurrentCount == 11) )
        {
          if(configPage2.nCylinders == 4) { triggerInfo.triggerToothAngle = 70; }
          else  { triggerInfo.triggerToothAngle = 70; }
        }
        else
        {
          if(configPage2.nCylinders == 4) { triggerInfo.triggerToothAngle = 110; }
          else  { triggerInfo.triggerToothAngle = 50; }
        }
      }

      //EXPERIMENTAL!
      //New ignition mode is ONLY available on 4g63 when the trigger angle is set to the stock value of 0.
      if( (configPage2.perToothIgn == true) && (configPage4.triggerAngle == 0) )
      {
        if( (configPage2.nCylinders == 4) && (currentStatus.advance > 0) )
        {
          int16_t crankAngle = ignitionLimits( triggerInfo.toothAngles[(triggerInfo.toothCurrentCount-1)] );

          //Handle non-sequential tooth counts
          if( (configPage4.sparkMode != IGN_MODE_SEQUENTIAL) && (triggerInfo.toothCurrentCount > configPage2.nCylinders) ) { checkPerToothTiming(crankAngle, (triggerInfo.toothCurrentCount-configPage2.nCylinders) ); }
          else { checkPerToothTiming(crankAngle, triggerInfo.toothCurrentCount); }
        }
      }
    } //Has sync
    else
    {
      triggerInfo.triggerSecFilterTime = 0;
      //New secondary method of determining sync
      if(READ_PRI_TRIGGER() == true)
      {
        if(READ_SEC_TRIGGER() == true) { triggerInfo.revolutionOne = true; }
        else { triggerInfo.revolutionOne = false; }
      }
      else
      {
        if( (READ_SEC_TRIGGER() == false) && (triggerInfo.revolutionOne == true) )
        {
          //Crank is low, cam is low and the crank pulse STARTED when the cam was high.
          if(configPage2.nCylinders == 4) { triggerInfo.toothCurrentCount = 1; } //Means we're at 5* BTDC on a 4G63 4 cylinder
          //else if(configPage2.nCylinders == 6) { triggerInfo.toothCurrentCount = 8; }
        }
        //If sequential is ever enabled, the below triggerInfo.toothCurrentCount will need to change:
        else if( (READ_SEC_TRIGGER() == true) && (triggerInfo.revolutionOne == true) )
        {
          //Crank is low, cam is high and the crank pulse STARTED when the cam was high.
          if(configPage2.nCylinders == 4) { triggerInfo.toothCurrentCount = 5; } //Means we're at 5* BTDC on a 4G63 4 cylinder
          else if(configPage2.nCylinders == 6) { triggerInfo.toothCurrentCount = 2; currentStatus.hasSync = true; } //Means we're at 45* ATDC on 6G72 6 cylinder
        }
      }
    }
  } //Filter time

}
void triggerSec_4G63(void)
{
  //byte crankState = READ_PRI_TRIGGER();
  //First filter is a duration based one to ensure the pulse was of sufficient length (time)
  //if(READ_SEC_TRIGGER()) { triggerInfo.secondaryLastToothTime1 = micros(); return; }
  if(currentStatus.hasSync == true)
  {
  //1166 is the time taken to cross 70 degrees at 10k rpm
  //if ( (micros() - triggerInfo.secondaryLastToothTime1) < triggerInfo.triggerSecFilterTime_duration ) { return; }
  //triggerInfo.triggerSecFilterTime_duration = (micros() - triggerInfo.secondaryLastToothTime1) >> 1;
  }


  triggerInfo.curTime2 = micros();
  triggerInfo.curGap2 = triggerInfo.curTime2 - triggerInfo.toothLastSecToothTime;
  if ( (triggerInfo.curGap2 >= triggerInfo.triggerSecFilterTime) )//|| (currentStatus.startRevolutions == 0) )
  {
    triggerInfo.toothLastSecToothTime = triggerInfo.curTime2;
    BIT_SET(triggerInfo.decoderState, BIT_DECODER_VALID_TRIGGER); //Flag this pulse as being a valid trigger (ie that it passed filters)
    //addToothLogEntry(triggerInfo.curGap, TOOTH_CAM_SECONDARY);

    triggerInfo.triggerSecFilterTime = triggerInfo.curGap2 >> 1; //Basic 50% filter for the secondary reading
    //More aggressive options:
    //62.5%:
    //triggerInfo.triggerSecFilterTime = (triggerInfo.curGap2 * 9) >> 5;
    //75%:
    //triggerInfo.triggerSecFilterTime = (triggerInfo.curGap2 * 6) >> 3;

    //if( (currentStatus.RPM < currentStatus.crankRPM) || (currentStatus.hasSync == false) )
    if( (currentStatus.hasSync == false) )
    {

      triggerInfo.triggerFilterTime = 1500; //If this is removed, can have trouble getting sync again after the engine is turned off (but ECU not reset).
      triggerInfo.triggerSecFilterTime = triggerInfo.triggerSecFilterTime >> 1; //Divide the secondary filter time by 2 again, making it 25%. Only needed when cranking
      if(READ_PRI_TRIGGER() == true)
      {
        if(configPage2.nCylinders == 4)
        {
          if(triggerInfo.toothCurrentCount == 8) { currentStatus.hasSync = true; } //Is 8 for sequential, was 4
        }
        else if(configPage2.nCylinders == 6)
        {
          if(triggerInfo.toothCurrentCount == 7) { currentStatus.hasSync = true; }
        }

      }
      else
      {
        if(configPage2.nCylinders == 4)
        {
          if(triggerInfo.toothCurrentCount == 5) { currentStatus.hasSync = true; } //Is 5 for sequential, was 1
        }
        //Cannot gain sync for 6 cylinder here.
      }
    }

    //if ( (micros() - triggerInfo.secondaryLastToothTime1) < triggerInfo.triggerSecFilterTime_duration && configPage2.useResync )
    if ( (currentStatus.RPM < currentStatus.crankRPM) || (configPage4.useResync == 1) )
    {
      if( (currentStatus.hasSync == true) && (configPage2.nCylinders == 4) )
      {
        triggerInfo.triggerSecFilterTime_duration = (micros() - triggerInfo.secondaryLastToothTime1) >> 1;
        if(READ_PRI_TRIGGER() == true)
        {
          //Whilst we're cranking and have sync, we need to watch for noise pulses.
          if(triggerInfo.toothCurrentCount != 8)
          {
            // This should never be true, except when there's noise
            currentStatus.hasSync = false;
            currentStatus.syncLossCounter++;
          }
          else { triggerInfo.toothCurrentCount = 8; } //Why? Just why?
        }
      } //Has sync and 4 cylinder
    } // Use resync or cranking
  } //Trigger filter
}


uint16_t getRPM_4G63(void)
{
  uint16_t tempRPM = 0;
  //During cranking, RPM is calculated 4 times per revolution, once for each rising/falling of the crank signal.
  //Because these signals aren't even (Alternating 110 and 70 degrees), this needs a special function
  if(currentStatus.hasSync == true)
  {
    if( (currentStatus.RPM < currentStatus.crankRPM)  )
    {
      int tempToothAngle;
      unsigned long toothTime;
      if( (triggerInfo.toothLastToothTime == 0) || (triggerInfo.toothLastMinusOneToothTime == 0) ) { tempRPM = 0; }
      else
      {
        noInterrupts();
        tempToothAngle = triggerInfo.triggerToothAngle;
        toothTime = (triggerInfo.toothLastToothTime - triggerInfo.toothLastMinusOneToothTime); //Note that trigger tooth angle changes between 70 and 110 depending on the last tooth that was seen (or 70/50 for 6 cylinders)
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
      //EXPERIMENTAL! Add/subtract RPM based on the last rpmDOT calc
      //tempRPM += (micros() - triggerInfo.toothOneTime) * currentStatus.rpmDOT
      MAX_STALL_TIME = revolutionTime << 1; //Set the stall time to be twice the current RPM. This is a safe figure as there should be no single revolution where this changes more than this
      if(MAX_STALL_TIME < 366667UL) { MAX_STALL_TIME = 366667UL; } //Check for 50rpm minimum
    }
  }

  return tempRPM;
}

int getCrankAngle_4G63(void)
{
    int crankAngle = 0;
    if(currentStatus.hasSync == true)
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

void triggerSetEndTeeth_4G63(void)
{
  if(configPage2.nCylinders == 4)
  {
    if(configPage4.sparkMode == IGN_MODE_SEQUENTIAL)
    {
      triggerInfo.ignition1EndTooth = 8;
      triggerInfo.ignition2EndTooth = 2;
      triggerInfo.ignition3EndTooth = 4;
      triggerInfo.ignition4EndTooth = 6;
    }
    else
    {
      triggerInfo.ignition1EndTooth = 4;
      triggerInfo.ignition2EndTooth = 2;
      triggerInfo.ignition3EndTooth = 4; //Not used
      triggerInfo.ignition4EndTooth = 2;
    }
  }
  if(configPage2.nCylinders == 6)
  {
    if(configPage4.sparkMode == IGN_MODE_SEQUENTIAL)
    {
      //This should never happen as 6 cylinder sequential not supported
      triggerInfo.ignition1EndTooth = 8;
      triggerInfo.ignition2EndTooth = 2;
      triggerInfo.ignition3EndTooth = 4;
      triggerInfo.ignition4EndTooth = 6;
    }
    else
    {
      triggerInfo.ignition1EndTooth = 6;
      triggerInfo.ignition2EndTooth = 2;
      triggerInfo.ignition3EndTooth = 4;
      triggerInfo.ignition4EndTooth = 2; //Not used
    }
  }
}
/** @} */


