

/*****************************************************************
 * Rover MEMS decoder
 * Covers multiple trigger wheels used interchanbably over the range of MEMS units
 * Specifically covers teeth patterns on the primary trigger (crank)
 * 3 gap 14 gap 2 gap 13 gap
 * 11 gap 5 gap 12 gap 4 gap
 * 2 gap 14 gap 3 gap 13 gap
 * 17 gap 17 gap
 *
 * Support no cam, single tooth Cam (or half moon cam), and multi tooth (5-3-2 teeth)
 *
 * @defgroup dec_rover_mems Rover MEMS all versions including T Series, O Series, Mini and K Series
 * @{
 */
volatile unsigned long roverMEMSTeethSeen = 0; // used for flywheel gap pattern matching

void triggerSetup_RoverMEMS()
{
  for(triggerInfo.toothOneTime = 0; triggerInfo.toothOneTime < 10; triggerInfo.toothOneTime++)   // repurpose variable temporarily to help clear triggerInfo.toothAngles.
    { triggerInfo.toothAngles[triggerInfo.toothOneTime] = 0; }// Repurpose triggerInfo.toothAngles to store data needed for this implementation.

  triggerInfo.triggerFilterTime = (MICROS_PER_SEC / (MAX_RPM / 60U * 36U)); //Trigger filter time is the shortest possible time (in uS) that there can be between crank teeth (ie at max RPM). Any pulses that occur faster than this time will be discarded as noise
  triggerInfo.triggerSecFilterTime = (MICROS_PER_SEC / (MAX_RPM / 60U)); // only 1 tooth on the wheel not 36

  configPage4.triggerTeeth = 36;
  triggerInfo.triggerToothAngle = 360 / configPage4.triggerTeeth; //The number of degrees that passes from tooth to tooth 360 / 36 theortical teeth
  triggerInfo.triggerActualTeeth = 36; //The number of physical teeth on the wheel. Need to fix now so we can identify the wheel on the first rotation and not risk a  type 1 wheel not being spotted
  triggerInfo.toothLastMinusOneToothTime = 0;
  triggerInfo.toothCurrentCount = 0; // current tooth
  triggerInfo.secondaryToothCount = 0;
  triggerInfo.secondaryLastToothCount = 0;
  triggerInfo.toothOneTime = 0;
  triggerInfo.toothOneMinusOneTime = 0;
  triggerInfo.revolutionOne=0;

  MAX_STALL_TIME = ((MICROS_PER_DEG_1_RPM/50U) * triggerInfo.triggerToothAngle * 2U); //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
  BIT_SET(triggerInfo.decoderState, BIT_DECODER_HAS_SECONDARY);

}

void triggerPri_RoverMEMS()
{
  triggerInfo.curTime = micros();
  triggerInfo.curGap = triggerInfo.curTime - triggerInfo.toothLastToothTime;

  if ( triggerInfo.curGap >= triggerInfo.triggerFilterTime ) //Pulses should never be less than triggerInfo.triggerFilterTime, so if they are it means a false trigger. (A 36-1 wheel at 8000pm will have triggers approx. every 200uS)
  {
    if( (triggerInfo.toothLastToothTime > 0) && (triggerInfo.toothLastMinusOneToothTime > 0) ) // have we seen more than 1 tooth so we start processing
    {
      //Begin the missing tooth detection
      triggerInfo.targetGap = (3 * (triggerInfo.toothLastToothTime - triggerInfo.toothLastMinusOneToothTime)) >> 1;  //Multiply by 1.5 (Checks for a gap 1.5x greater than the last one) (Uses bitshift to multiply by 3 then divide by 2. Much faster than multiplying by 1.5)
      //currentStatus.hasSync = true;
      if ( triggerInfo.curGap > triggerInfo.targetGap) // we've found a gap
      {
        roverMEMSTeethSeen = roverMEMSTeethSeen << 2; // add the space for the gap and the tooth we've just seen so shift 2 bits
        roverMEMSTeethSeen++; // add the tooth seen to the variable
        triggerInfo.toothCurrentCount++; // Increment the tooth counter on the wheel (used to spot a revolution and trigger igition timing)

        // the missing tooth gap messing up timing as it appears in different parts of the cycle. Don't update setFilter as it would be wrong with the gap
        triggerInfo.toothCurrentCount++;
      }
      else //Regular (non-missing) tooth so update things
      {
        roverMEMSTeethSeen = roverMEMSTeethSeen << 1; // make a space, shift the bits 1 place to the left
        roverMEMSTeethSeen++; // add the tooth seen
        triggerInfo.toothCurrentCount++; //Increment the tooth counter on the wheel (used to spot a revolution)
        setFilter(triggerInfo.curGap);
      }

      // reduce checks to minimise cpu load when looking for key point to identify where we are on the wheel
      if( triggerInfo.toothCurrentCount >= triggerInfo.triggerActualTeeth )
      {                           //12345678901234567890123456789012
        if( roverMEMSTeethSeen == 0b11111101111111011111111110111111) // Binary pattern for trigger pattern 9-7-10-6- (#5)
        {
          if(triggerInfo.toothAngles[ID_TOOTH_PATTERN] != 5)
          {
            //teeth to skip when calculating RPM as they've just had a gap
            triggerInfo.toothAngles[SKIP_TOOTH1] = 1;
            triggerInfo.toothAngles[SKIP_TOOTH2] = 11;
            triggerInfo.toothAngles[SKIP_TOOTH3] = 19;
            triggerInfo.toothAngles[SKIP_TOOTH4] = 30;
            triggerInfo.toothAngles[ID_TOOTH_PATTERN] = 5;
            configPage4.triggerMissingTeeth = 4; // this could be read in from the config file, but people could adjust it.
            triggerInfo.triggerActualTeeth = 36; // should be 32 if not hacking toothcounter
          }
          triggerRoverMEMSCommon();
        }                             //123456789012345678901234567890123456
        else if( roverMEMSTeethSeen == 0b11011101111111111111101101111111) // Binary pattern for trigger pattern 3-14-2-13- (#4)
        {
          if(triggerInfo.toothAngles[ID_TOOTH_PATTERN] != 4)
          {
            //teeth to skip when calculating RPM as they've just had a gap
            triggerInfo.toothAngles[SKIP_TOOTH1] = 8;
            triggerInfo.toothAngles[SKIP_TOOTH2] = 11;
            triggerInfo.toothAngles[SKIP_TOOTH3] = 25;
            triggerInfo.toothAngles[SKIP_TOOTH4] = 27;
            triggerInfo.toothAngles[ID_TOOTH_PATTERN] = 4;
            configPage4.triggerMissingTeeth = 4; // this could be read in from the config file, but people could adjust it.
            triggerInfo.triggerActualTeeth = 36; // should be 32 if not hacking toothcounter
          }
          triggerRoverMEMSCommon();
        }                             //123456789012345678901234567890123456
        else if(roverMEMSTeethSeen == 0b11011011111111111111011101111111) // Binary pattern for trigger pattern 2-14-3-13- (#3)
        {
          if(triggerInfo.toothAngles[ID_TOOTH_PATTERN] != 3)
          {
            //teeth to skip when calculating RPM as they've just had a gap
            triggerInfo.toothAngles[SKIP_TOOTH1] = 8;
            triggerInfo.toothAngles[SKIP_TOOTH2] = 10;
            triggerInfo.toothAngles[SKIP_TOOTH3] = 24;
            triggerInfo.toothAngles[SKIP_TOOTH4] = 27;
            triggerInfo.toothAngles[ID_TOOTH_PATTERN] = 3;
            configPage4.triggerMissingTeeth = 4; // this could be read in from the config file, but people could adjust it.
            triggerInfo.triggerActualTeeth = 36; // should be 32 if not hacking toothcounter
          }
          triggerRoverMEMSCommon();
        }                             //12345678901234567890123456789012
        else if(roverMEMSTeethSeen == 0b11111101111101111111111110111101) // Binary pattern for trigger pattern 11-5-12-4- (#2)
        {
          if(triggerInfo.toothAngles[ID_TOOTH_PATTERN] != 2)
          {
            //teeth to skip when calculating RPM as they've just had a gap
            triggerInfo.toothAngles[SKIP_TOOTH1] = 1;
            triggerInfo.toothAngles[SKIP_TOOTH2] = 12;
            triggerInfo.toothAngles[SKIP_TOOTH3] = 17;
            triggerInfo.toothAngles[SKIP_TOOTH4] = 29;
            triggerInfo.toothAngles[ID_TOOTH_PATTERN] = 2;
            configPage4.triggerMissingTeeth = 4; // this could be read in from the config file, but people could adjust it.
            triggerInfo.triggerActualTeeth = 36; // should be 32 if not hacking toothcounter
          }
          triggerRoverMEMSCommon();
        }                             //12345678901234567890123456789012
        else if(roverMEMSTeethSeen == 0b11111111111101111111111111111101) // Binary pattern for trigger pattern 17-17- (#1)
        {
          if(triggerInfo.toothAngles[ID_TOOTH_PATTERN] != 1)
          {
            //teeth to skip when calculating RPM as they've just had a gap
            triggerInfo.toothAngles[SKIP_TOOTH1] = 1;
            triggerInfo.toothAngles[SKIP_TOOTH2] = 18;
            triggerInfo.toothAngles[ID_TOOTH_PATTERN] = 1;
            configPage4.triggerMissingTeeth = 2; // this should be read in from the config file, but people could adjust it.
            triggerInfo.triggerActualTeeth = 36; // should be 34 if not hacking toothcounter
          }
          triggerRoverMEMSCommon();
        }
        else if(triggerInfo.toothCurrentCount > triggerInfo.triggerActualTeeth+1) // no patterns match after a rotation when we only need 32 teeth to match, we've lost sync
        {
          currentStatus.hasSync = false;
          BIT_CLEAR(currentStatus.status3, BIT_STATUS3_HALFSYNC);
          currentStatus.syncLossCounter++;
        }
      }
    }

    triggerInfo.toothLastMinusOneToothTime = triggerInfo.toothLastToothTime;
    triggerInfo.toothLastToothTime = triggerInfo.curTime;

    //NEW IGNITION MODE
    if( (configPage2.perToothIgn == true) && (!BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK)) )
    {
      int16_t crankAngle = ( (triggerInfo.toothCurrentCount-1) * triggerInfo.triggerToothAngle ) + configPage4.triggerAngle;
      crankAngle = ignitionLimits(crankAngle);
      if( (configPage4.sparkMode == IGN_MODE_SEQUENTIAL) && (triggerInfo.revolutionOne == true))
      { crankAngle += 360; checkPerToothTiming(crankAngle, (configPage4.triggerTeeth + triggerInfo.toothCurrentCount)); }
      else
      { checkPerToothTiming(crankAngle, triggerInfo.toothCurrentCount); }
    }
  }

}


static void triggerRoverMEMSCommon(void)
{
  // pattern 1 isn't unique & if we don't have a cam we need special code to identify if we're tooth 18 or 36 - this allows batch injection but not spark to run
  // as we have to be greater than 18 teeth when using the cam this code also works for that.
  if( triggerInfo.toothCurrentCount > 18)
  {
    triggerInfo.toothCurrentCount = 1;
    triggerInfo.toothOneMinusOneTime = triggerInfo.toothOneTime;
    triggerInfo.toothOneTime = triggerInfo.curTime;
    triggerInfo.revolutionOne = !triggerInfo.revolutionOne; //Flip sequential revolution tracker
  }

  if( (configPage4.sparkMode == IGN_MODE_SEQUENTIAL) || (configPage2.injLayout == INJ_SEQUENTIAL) )
  {
    //If either fuel or ignition is sequential, only declare sync if the cam tooth has been seen OR if the missing wheel is on the cam
    if( (triggerInfo.secondaryToothCount > 0) || (configPage4.TrigSpeed == CAM_SPEED) )
    {
      currentStatus.hasSync = true;
      BIT_CLEAR(currentStatus.status3, BIT_STATUS3_HALFSYNC); //the engine is fully synced so clear the Half Sync bit
      if(configPage4.trigPatternSec == SEC_TRIGGER_SINGLE) { triggerInfo.secondaryToothCount = 0; } //Reset the secondary tooth counter to prevent it overflowing
    }
    else if(currentStatus.hasSync != true)
    { BIT_SET(currentStatus.status3, BIT_STATUS3_HALFSYNC); } //If there is primary trigger but no secondary we only have half sync.
  }
  else { currentStatus.hasSync = false;  BIT_SET(currentStatus.status3, BIT_STATUS3_HALFSYNC); } //If nothing is using sequential, we  set half sync bit

  currentStatus.startRevolutions++;
}




int getCrankAngle_RoverMEMS()
{
    //This is the current angle ATDC the engine is at. This is the last known position based on what tooth was last 'seen'. It is only accurate to the resolution of the trigger wheel (Eg 36-1 is 10 degrees)
    unsigned long temptoothLastToothTime;
    int temptoothCurrentCount;
    bool temprevolutionOne;
    //Grab some variables that are used in the trigger code and assign them to temp variables.
    noInterrupts();
    temptoothCurrentCount = triggerInfo.toothCurrentCount;
    temprevolutionOne = triggerInfo.revolutionOne;
    temptoothLastToothTime = triggerInfo.toothLastToothTime;
    interrupts();

    int crankAngle = ((temptoothCurrentCount - 1) * triggerInfo.triggerToothAngle) + configPage4.triggerAngle; //Number of teeth that have passed since tooth 1, multiplied by the angle each tooth represents, plus the angle that tooth 1 is ATDC. This gives accuracy only to the nearest tooth.

    //Sequential check (simply sets whether we're on the first or 2nd revolution of the cycle)
    if ( (temprevolutionOne == true) && (configPage4.TrigSpeed == CRANK_SPEED) ) { crankAngle += 360; }

    triggerInfo.lastCrankAngleCalc = micros();
    triggerInfo.elapsedTime = (triggerInfo.lastCrankAngleCalc - temptoothLastToothTime);
    crankAngle += timeToAngleDegPerMicroSec(triggerInfo.elapsedTime);

    if (crankAngle >= 720) { crankAngle -= 720; }
    if (crankAngle < 0) { crankAngle += CRANK_ANGLE_MAX; }

    return crankAngle;
}

void triggerSec_RoverMEMS()
{
  triggerInfo.curTime2 = micros();
  triggerInfo.curGap2 = triggerInfo.curTime2 - triggerInfo.toothLastSecToothTime;

  //Safety check for initial startup
  if( (triggerInfo.toothLastSecToothTime == 0) )
  {
    triggerInfo.targetGap2 = triggerInfo.curGap * 2;
    triggerInfo.curGap2 = 0;
    triggerInfo.toothLastSecToothTime = triggerInfo.curTime2;
  }

  if ( triggerInfo.curGap2 >= triggerInfo.triggerSecFilterTime )
  {
    triggerInfo.secondaryToothCount++;
    triggerInfo.toothLastSecToothTime = triggerInfo.curTime2;


    //Record the VVT Angle
    if( configPage6.vvtEnabled > 0 &&
        ( (configPage4.trigPatternSec == SEC_TRIGGER_SINGLE) ||
          (configPage4.trigPatternSec == SEC_TRIGGER_5_3_2 && triggerInfo.secondaryToothCount == 6 ) ) )
    {
      int16_t curAngle;
      curAngle = getCrankAngle();
      while(curAngle > 360) { curAngle -= 360; }
      curAngle -= configPage4.triggerAngle; //Value at TDC
      if( configPage6.vvtMode == VVT_MODE_CLOSED_LOOP ) { curAngle -= configPage10.vvtCLMinAng; }

      currentStatus.vvt1Angle = curAngle;
    }

    if(configPage4.trigPatternSec == SEC_TRIGGER_SINGLE)
    {
      //Standard single tooth cam trigger
      triggerInfo.revolutionOne = true;
      triggerInfo.triggerSecFilterTime = triggerInfo.curGap2 >> 1; //Next secondary filter is half the current gap
    }
    else if (configPage4.trigPatternSec == SEC_TRIGGER_5_3_2) // multi tooth cam
    {
      if (triggerInfo.curGap2 < triggerInfo.targetGap2) // ie normal tooth sized gap, not a single or double gap
      {
        triggerInfo.triggerSecFilterTime = triggerInfo.curGap2 >> 1; //Next secondary filter is half the current gap
        triggerInfo.targetGap2 = (3 * (triggerInfo.curGap2)) >> 1;  //Multiply by 1.5 (Checks for a gap 1.5x greater than the last one) (Uses bitshift to multiply by 3 then divide by 2. Much faster than multiplying by 1.5)
      }
      else
      {
        // gap either single or double - nb remember we've got the tooth after the gap, so on the 5 tooth pattern we'll see here tooth 6
        if(triggerInfo.secondaryToothCount == 6)
        {
          // if we've got the tooth after the gap from reading 5 teeth we're on cycle 360-720 & tooth 18-36
          triggerInfo.revolutionOne = false;
          if(triggerInfo.toothCurrentCount < 19)
          {
            triggerInfo.toothCurrentCount += 18;
          }
        }
        else if (triggerInfo.secondaryToothCount == 4)
        {
          // we've got the tooth after the gap from reading 3 teeth we're on cycle 0-360 & tooth 1-18
          triggerInfo.revolutionOne = true;
          if(triggerInfo.toothCurrentCount > 17)
          {
            triggerInfo.toothCurrentCount -= 18;
          }
        }
        else if (triggerInfo.secondaryToothCount == 3)
        {
          // if we've got the tooth after the gap from reading 2 teeth we're on cycle 0-360 & tooth 18-36
          triggerInfo.revolutionOne = true;
          if(triggerInfo.toothCurrentCount < 19)
          {
            triggerInfo.toothCurrentCount += 18;
          }
        }
        triggerInfo.secondaryToothCount = 1; // as we've had a gap we need to reset to this being the first tooth after the gap
      }
    }
  } //Trigger filter
}

uint16_t getRPM_RoverMEMS()
{
  uint16_t tempRPM = 0;

  if( currentStatus.RPM < currentStatus.crankRPM)
  {
    if( (triggerInfo.toothCurrentCount != (unsigned int) triggerInfo.toothAngles[SKIP_TOOTH1]) &&
        (triggerInfo.toothCurrentCount != (unsigned int) triggerInfo.toothAngles[SKIP_TOOTH2]) &&
        (triggerInfo.toothCurrentCount != (unsigned int) triggerInfo.toothAngles[SKIP_TOOTH3]) &&
        (triggerInfo.toothCurrentCount != (unsigned int) triggerInfo.toothAngles[SKIP_TOOTH4]) )
    { tempRPM = crankingGetRPM(36, CRANK_SPEED); }
    else
    { tempRPM = currentStatus.RPM; } //Can't do per tooth RPM as the missing tooth messes the calculation
  }
  else
  { tempRPM = stdGetRPM(CRANK_SPEED); }
  return tempRPM;
}


void triggerSetEndTeeth_RoverMEMS()
{
  //Temp variables are used here to avoid potential issues if a trigger interrupt occurs part way through this function
  int16_t tempIgnitionEndTooth[5]; // cheating with the array - location 1 is spark 1, location 0 not used.
  int16_t toothAdder = 0;

  if( (configPage4.sparkMode == IGN_MODE_SEQUENTIAL) && (configPage4.TrigSpeed == CRANK_SPEED) ) { toothAdder = 36; }

  tempIgnitionEndTooth[1] = ( (ignition1EndAngle - configPage4.triggerAngle) / (int16_t)(10) ) - 1;
  if(tempIgnitionEndTooth[1] > (36 + toothAdder)) { tempIgnitionEndTooth[1] -= (36 + toothAdder); }
  if(tempIgnitionEndTooth[1] <= 0) { tempIgnitionEndTooth[1] += (36 + toothAdder); }
  if(tempIgnitionEndTooth[1] > (36 + toothAdder)) { tempIgnitionEndTooth[1] = (36 + toothAdder); }

  tempIgnitionEndTooth[2] = ( (ignition2EndAngle - configPage4.triggerAngle) / (int16_t)(10) ) - 1;
  if(tempIgnitionEndTooth[2] > (36 + toothAdder)) { tempIgnitionEndTooth[2] -= (36 + toothAdder); }
  if(tempIgnitionEndTooth[2] <= 0) { tempIgnitionEndTooth[2] += (36 + toothAdder); }
  if(tempIgnitionEndTooth[2] > (36 + toothAdder)) { tempIgnitionEndTooth[2] = (36 + toothAdder); }

  tempIgnitionEndTooth[3] = ( (ignition3EndAngle - configPage4.triggerAngle) / (int16_t)(10) ) - 1;
  if(tempIgnitionEndTooth[3] > (36 + toothAdder)) { tempIgnitionEndTooth[3] -= (36 + toothAdder); }
  if(tempIgnitionEndTooth[3] <= 0) { tempIgnitionEndTooth[3] += (36 + toothAdder); }
  if(tempIgnitionEndTooth[3] > (36 + toothAdder)) { tempIgnitionEndTooth[3] = (36 + toothAdder); }

  tempIgnitionEndTooth[4] = ( (ignition4EndAngle - configPage4.triggerAngle) / (int16_t)(10) ) - 1;
  if(tempIgnitionEndTooth[4] > (36 + toothAdder)) { tempIgnitionEndTooth[4] -= (36 + toothAdder); }
  if(tempIgnitionEndTooth[4] <= 0) { tempIgnitionEndTooth[4] += (36 + toothAdder); }
  if(tempIgnitionEndTooth[4] > (36 + toothAdder)) { tempIgnitionEndTooth[4] = (36 + toothAdder); }

  // take into account the missing teeth on the Rover flywheels
  int tempCount=0;

  if(configPage4.sparkMode == IGN_MODE_SEQUENTIAL)
  {
    // check the calculated trigger tooth exists, if it doesn't use the previous tooth
    // nb the triggerInfo.toothAngles[x] holds the tooth after the gap, hence the '-1' to see if it matches a gap

    for(tempCount=1;tempCount <5;tempCount++)
    {
      if(tempIgnitionEndTooth[tempCount] == (triggerInfo.toothAngles[1]) || tempIgnitionEndTooth[tempCount] == (triggerInfo.toothAngles[2]) ||
         tempIgnitionEndTooth[tempCount] == (triggerInfo.toothAngles[3]) || tempIgnitionEndTooth[tempCount] == (triggerInfo.toothAngles[4]) ||
         tempIgnitionEndTooth[tempCount] == (36 + triggerInfo.toothAngles[1]) || tempIgnitionEndTooth[tempCount] == (36 + triggerInfo.toothAngles[2]) ||
         tempIgnitionEndTooth[tempCount] == (36 + triggerInfo.toothAngles[3]) || tempIgnitionEndTooth[tempCount] == (36 + triggerInfo.toothAngles[4])  )
      { tempIgnitionEndTooth[tempCount]--; }
    }
  }
  else
  {
    for(tempCount=1;tempCount<5;tempCount++)
    {
      if(tempIgnitionEndTooth[tempCount] == (triggerInfo.toothAngles[1]) || tempIgnitionEndTooth[tempCount] == (triggerInfo.toothAngles[2]) )
      { tempIgnitionEndTooth[tempCount]--; }
    }
  }


  triggerInfo.ignition1EndTooth = tempIgnitionEndTooth[1];
  triggerInfo.ignition2EndTooth = tempIgnitionEndTooth[2];
  triggerInfo.ignition3EndTooth = tempIgnitionEndTooth[3];
  triggerInfo.ignition4EndTooth = tempIgnitionEndTooth[4];
}
/** @} */


