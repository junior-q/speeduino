

/** Chrysler NGC - a dedicated decoder for vehicles with 4, 6 and 8 cylinder NGC pattern.
4-cyl: 36+2-2 crank wheel and 7 tooth cam
6-cyl: 36-2+2 crank wheel and 12 tooth cam in 6 groups
8-cyl: 36-2+2 crank wheel and 15 tooth cam in 8 groups
The crank decoder uses the polarity of the missing teeth to determine position
The 4-cyl cam decoder uses the polarity of the missing teeth to determine position
The 6 and 8-cyl cam decoder uses the amount of teeth in the two previous groups of teeth to determine position
* @defgroup dec Chrysler NGC - 4, 6 and 8-cylinder
* @{
*/

void triggerSetup_NGC(void)
{
  BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_2ND_DERIV);
  BIT_SET(triggerInfo.decoderState, BIT_DECODER_IS_SEQUENTIAL);
  BIT_SET(triggerInfo.decoderState, BIT_DECODER_HAS_SECONDARY);

  //Primary trigger
  configPage4.triggerTeeth = 36; //The number of teeth on the wheel incl missing teeth.
  triggerInfo.triggerToothAngle = 10; //The number of degrees that passes from tooth to tooth
  triggerInfo.triggerFilterTime = MICROS_PER_SEC / (MAX_RPM/60U) / (360U/triggerInfo.triggerToothAngle); //Trigger filter time is the shortest possible time (in uS) that there can be between crank teeth (ie at max RPM). Any pulses that occur faster than this time will be discarded as noise
  triggerInfo.toothCurrentCount = 0;
  triggerInfo.toothOneTime = 0;
  triggerInfo.toothOneMinusOneTime = 0;
  triggerInfo.toothLastMinusOneToothTime = 0;
  triggerInfo.toothLastToothRisingTime = 0;
  MAX_STALL_TIME = ((MICROS_PER_DEG_1_RPM/50U) * triggerInfo.triggerToothAngle * 2U ); //Minimum 50rpm. (3333uS is the time per degree at 50rpm)

  //Secondary trigger
  if (configPage2.nCylinders == 4) {
    triggerInfo.triggerSecFilterTime = (MICROS_PER_SEC / (MAX_RPM/60U) / (360U/36U) * 2U); //Two nearest edges are 36 degrees apart. Multiply by 2 for half cam speed.
  } else {
    triggerInfo.triggerSecFilterTime = (MICROS_PER_SEC / (MAX_RPM/60U) / (360U/21U) * 2U); //Two nearest edges are 21 degrees apart. Multiply by 2 for half cam speed.
  }
  triggerInfo.secondaryToothCount = 0;
  triggerInfo.toothSystemCount = 0;
  triggerInfo.toothLastSecToothRisingTime = 0;
  triggerInfo.toothLastSecToothTime = 0;
  triggerInfo.toothLastMinusOneSecToothTime = 0;

  //triggerInfo.toothAngles is reused to store the cam pattern, only used for 6 and 8 cylinder pattern
  if (configPage2.nCylinders == 6) {
    triggerInfo.toothAngles[0] = 1; // Pos 0 is required to be the same as group 6 for easier math
    triggerInfo.toothAngles[1] = 3; // Group 1 ...
    triggerInfo.toothAngles[2] = 1;
    triggerInfo.toothAngles[3] = 2;
    triggerInfo.toothAngles[4] = 3;
    triggerInfo.toothAngles[5] = 2;
    triggerInfo.toothAngles[6] = 1;
    triggerInfo.toothAngles[7] = 3; // Pos 7 is required to be the same as group 1 for easier math
  }
  else if (configPage2.nCylinders == 8) {
    triggerInfo.toothAngles[0] = 3; // Pos 0 is required to be the same as group 8 for easier math
    triggerInfo.toothAngles[1] = 1; // Group 1 ...
    triggerInfo.toothAngles[2] = 1;
    triggerInfo.toothAngles[3] = 2;
    triggerInfo.toothAngles[4] = 3;
    triggerInfo.toothAngles[5] = 2;
    triggerInfo.toothAngles[6] = 2;
    triggerInfo.toothAngles[7] = 1;
    triggerInfo.toothAngles[8] = 3;
    triggerInfo.toothAngles[9] = 1; // Pos 9 is required to be the same as group 1 for easier math
  }
#ifdef USE_LIBDIVIDE
  divtriggerInfo.triggerToothAngle = libdivide::libdivide_s16_gen(triggerInfo.triggerToothAngle);
#endif
}

void triggerPri_NGC(void)
{
  triggerInfo.curTime = micros();
  // We need to know the polarity of the missing tooth to determine position
  if (READ_PRI_TRIGGER() == HIGH) {
    triggerInfo.toothLastToothRisingTime = triggerInfo.curTime;
    return;
  }

  triggerInfo.curGap = triggerInfo.curTime - triggerInfo.toothLastToothTime;
  if ( triggerInfo.curGap >= triggerInfo.triggerFilterTime ) //Pulses should never be less than triggerInfo.triggerFilterTime, so if they are it means a false trigger.
  {
    triggerInfo.toothCurrentCount++;
    BIT_SET(triggerInfo.decoderState, BIT_DECODER_VALID_TRIGGER); //Flag this pulse as being a valid trigger (ie that it passed filters)
    bool isMissingTooth = false;

    if ( triggerInfo.toothLastToothTime > 0 && triggerInfo.toothLastMinusOneToothTime > 0 ) { //Make sure we haven't enough tooth information to calculate missing tooth length

      //Only check for missing tooth if we expect this one to be it or if we haven't found one yet
      if (triggerInfo.toothCurrentCount == 17 || triggerInfo.toothCurrentCount == 35 || ( currentStatus.hasSync == false && BIT_CHECK(currentStatus.status3, BIT_STATUS3_HALFSYNC) == false) ) {
        //If the time between the current tooth and the last is greater than 2x the time between the last tooth and the tooth before that, we make the assertion that we must be at the first tooth after the gap
        if (triggerInfo.curGap > ( (triggerInfo.toothLastToothTime - triggerInfo.toothLastMinusOneToothTime) * 2 ) )
        {
          isMissingTooth = true; //Missing tooth detected
          triggerInfo.triggerFilterTime = 0; //This is used to prevent a condition where serious intermittent signals (Eg someone furiously plugging the sensor wire in and out) can leave the filter in an unrecoverable state
          BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_TOOTH_ANG_CORRECT); //The tooth angle is double at this point

          // Figure out the polarity of the missing tooth by comparing how far ago the last tooth rose
          if ((triggerInfo.toothLastToothRisingTime - triggerInfo.toothLastToothTime) < (triggerInfo.curTime - triggerInfo.toothLastToothRisingTime)) {
            //Just passed the HIGH missing tooth
            triggerInfo.toothCurrentCount = 1;

            triggerInfo.toothOneMinusOneTime = triggerInfo.toothOneTime;
            triggerInfo.toothOneTime = triggerInfo.curTime;

            if (currentStatus.hasSync == true) { currentStatus.startRevolutions++; }
            else { currentStatus.startRevolutions = 0; }
          }
          else {
            //Just passed the first tooth after the LOW missing tooth
            triggerInfo.toothCurrentCount = 19;
          }

          //If Sequential fuel or ignition is in use, further checks are needed before determining sync
          if( (configPage4.sparkMode == IGN_MODE_SEQUENTIAL) || (configPage2.injLayout == INJ_SEQUENTIAL) )
          {
            // Verify the tooth counters are valid and use this to determine current revolution
            if (
              ( configPage2.nCylinders == 4 && ( (triggerInfo.toothCurrentCount == 1 && (triggerInfo.secondaryToothCount == 1 || triggerInfo.secondaryToothCount == 2) ) || (triggerInfo.toothCurrentCount == 19 && triggerInfo.secondaryToothCount == 4) ) ) ||
              ( configPage2.nCylinders == 6 && ( (triggerInfo.toothCurrentCount == 1 && (triggerInfo.toothSystemCount == 1    || triggerInfo.toothSystemCount == 2) )    || (triggerInfo.toothCurrentCount == 19 && (triggerInfo.toothSystemCount == 2 || triggerInfo.toothSystemCount == 3) ) ) ) ||
              ( configPage2.nCylinders == 8 && ( (triggerInfo.toothCurrentCount == 1 && (triggerInfo.toothSystemCount == 1    || triggerInfo.toothSystemCount == 2) )    || (triggerInfo.toothCurrentCount == 19 && (triggerInfo.toothSystemCount == 3 || triggerInfo.toothSystemCount == 4) ) ) ) )
            {
              triggerInfo.revolutionOne = false;
              currentStatus.hasSync = true;
              BIT_CLEAR(currentStatus.status3, BIT_STATUS3_HALFSYNC); //the engine is fully synced so clear the Half Sync bit
            }
            else if (
              ( configPage2.nCylinders == 4 && ( (triggerInfo.toothCurrentCount == 1 && triggerInfo.secondaryToothCount == 5)                          || (triggerInfo.toothCurrentCount == 19 && triggerInfo.secondaryToothCount == 7) ) ) ||
              ( configPage2.nCylinders == 6 && ( (triggerInfo.toothCurrentCount == 1 && (triggerInfo.toothSystemCount == 4 || triggerInfo.toothSystemCount == 5) ) || (triggerInfo.toothCurrentCount == 19 && (triggerInfo.toothSystemCount == 5 || triggerInfo.toothSystemCount == 6) ) ) ) ||
              ( configPage2.nCylinders == 8 && ( (triggerInfo.toothCurrentCount == 1 && (triggerInfo.toothSystemCount == 5 || triggerInfo.toothSystemCount == 6) ) || (triggerInfo.toothCurrentCount == 19 && (triggerInfo.toothSystemCount == 7 || triggerInfo.toothSystemCount == 8) ) ) ) )
            {
              triggerInfo.revolutionOne = true;
              currentStatus.hasSync = true;
              BIT_CLEAR(currentStatus.status3, BIT_STATUS3_HALFSYNC); //the engine is fully synced so clear the Half Sync bit
            }
            // If tooth counters are not valid, set half sync bit
            else {
              if (currentStatus.hasSync == true) { currentStatus.syncLossCounter++; }
              currentStatus.hasSync = false;
              BIT_SET(currentStatus.status3, BIT_STATUS3_HALFSYNC); //If there is primary trigger but no secondary we only have half sync.
            }
          }
          else { currentStatus.hasSync = true;  BIT_CLEAR(currentStatus.status3, BIT_STATUS3_HALFSYNC); } //If nothing is using sequential, we have sync and also clear half sync bit

        }
        else {
          // If we have found a missing tooth and don't get the next one at the correct tooth we end up here -> Resync
          if (currentStatus.hasSync == true) { currentStatus.syncLossCounter++; }
          currentStatus.hasSync = false;
          BIT_CLEAR(currentStatus.status3, BIT_STATUS3_HALFSYNC);
        }
      }

      if(isMissingTooth == false)
      {
        //Regular (non-missing) tooth
        setFilter(triggerInfo.curGap);
        BIT_SET(triggerInfo.decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);
      }
    }

    if (isMissingTooth == true) { // If we have a missing tooth, copy the gap from the previous tooth as that is the correct normal tooth length
      triggerInfo.toothLastMinusOneToothTime = triggerInfo.curTime - (triggerInfo.toothLastToothTime - triggerInfo.toothLastMinusOneToothTime);
    }
    else {
      triggerInfo.toothLastMinusOneToothTime = triggerInfo.toothLastToothTime;
    }
    triggerInfo.toothLastToothTime = triggerInfo.curTime;

    //NEW IGNITION MODE
    if( (configPage2.perToothIgn == true) && (BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) == false) )
    {
      int16_t crankAngle = ( (triggerInfo.toothCurrentCount-1) * triggerInfo.triggerToothAngle ) + configPage4.triggerAngle;
      crankAngle = ignitionLimits(crankAngle);
      if( (configPage4.sparkMode == IGN_MODE_SEQUENTIAL) && (triggerInfo.revolutionOne == true) && (configPage4.TrigSpeed == CRANK_SPEED) )
      {
        crankAngle += 360;
        checkPerToothTiming(crankAngle, (configPage4.triggerTeeth + triggerInfo.toothCurrentCount));
      }
    else{ checkPerToothTiming(crankAngle, triggerInfo.toothCurrentCount); }
    }
  }
}

void triggerSec_NGC4(void)
{
  //Only check the cam wheel for sequential operation
  if( configPage4.sparkMode != IGN_MODE_SEQUENTIAL && configPage2.injLayout != INJ_SEQUENTIAL ) {
    return;
  }

  triggerInfo.curTime2 = micros();

  // We need to know the polarity of the missing tooth to determine position
  if (READ_SEC_TRIGGER() == HIGH) {
    triggerInfo.toothLastSecToothRisingTime = triggerInfo.curTime2;
    return;
  }

  triggerInfo.curGap2 = triggerInfo.curTime2 - triggerInfo.toothLastSecToothTime;

  if ( triggerInfo.curGap2 > triggerInfo.triggerSecFilterTime )
  {
    if ( triggerInfo.toothLastSecToothTime > 0 && triggerInfo.toothLastMinusOneSecToothTime > 0 ) //Make sure we have enough tooth information to calculate tooth lengths
    {
      if (triggerInfo.secondaryToothCount > 0) { triggerInfo.secondaryToothCount++; }

      if (triggerInfo.curGap2 >= ((3 * (triggerInfo.toothLastSecToothTime - triggerInfo.toothLastMinusOneSecToothTime)) >> 1)) // Check if we have a bigger gap, that is a long tooth
      {
        // Check long tooth polarity
        if ((triggerInfo.toothLastSecToothRisingTime - triggerInfo.toothLastSecToothTime) < (triggerInfo.curTime2 - triggerInfo.toothLastSecToothRisingTime)) {
          //Just passed the HIGH missing tooth
          if ( triggerInfo.secondaryToothCount == 0 || triggerInfo.secondaryToothCount == 8 ) { triggerInfo.secondaryToothCount = 1; } // synced
          else if (triggerInfo.secondaryToothCount > 0) { triggerInfo.secondaryToothCount = 0; } //Any other number of teeth seen means we missed something or something extra was seen so attempt resync.
        }
        else {
          //Just passed the first tooth after the LOW missing tooth
          if ( triggerInfo.secondaryToothCount == 0 || triggerInfo.secondaryToothCount == 5 ) { triggerInfo.secondaryToothCount = 5; }
          else if (triggerInfo.secondaryToothCount > 0) { triggerInfo.secondaryToothCount = 0; }
        }

        triggerInfo.triggerSecFilterTime = 0; //This is used to prevent a condition where serious intermittent signals (Eg someone furiously plugging the sensor wire in and out) can leave the filter in an unrecoverable state
      }
      else if (triggerInfo.secondaryToothCount > 0) {
        triggerInfo.triggerSecFilterTime = triggerInfo.curGap2 >> 2; //Set filter at 25% of the current speed. Filter can only be recalc'd for the regular teeth, not the missing one.
      }

    }

    triggerInfo.toothLastMinusOneSecToothTime = triggerInfo.toothLastSecToothTime;
    triggerInfo.toothLastSecToothTime = triggerInfo.curTime2;
  }
}

#define secondaryToothLastCount triggerInfo.checkSyncToothCount

void triggerSec_NGC68(void)
{
  //Only check the cam wheel for sequential operation
  if( configPage4.sparkMode != IGN_MODE_SEQUENTIAL && configPage2.injLayout != INJ_SEQUENTIAL ) {
    return;
  }

  triggerInfo.curTime2 = micros();

  triggerInfo.curGap2 = triggerInfo.curTime2 - triggerInfo.toothLastSecToothTime;

  if ( triggerInfo.curGap2 > triggerInfo.triggerSecFilterTime )
  {
    if ( triggerInfo.toothLastSecToothTime > 0 && triggerInfo.toothLastToothTime > 0 && triggerInfo.toothLastMinusOneToothTime > 0 ) //Make sure we have enough tooth information to calculate tooth lengths
    {
      /* Cam wheel can have a single tooth in a group which can screw up the "triggerInfo.targetGap" calculations
         Instead use primary wheel tooth gap as comparison as those values are always correct. 2.1 primary teeth are the same duration as one secondary tooth. */
      if (triggerInfo.curGap2 >= (3 * (triggerInfo.toothLastToothTime - triggerInfo.toothLastMinusOneToothTime) ) ) // Check if we have a bigger gap, that is missing teeth
      {
        //triggerInfo.toothSystemCount > 0 means we have cam sync and identifies which group we have synced with
        //triggerInfo.toothAngles is reused to store the cam pattern
        if (triggerInfo.secondaryToothCount > 0 && secondaryToothLastCount > 0) { // Only check for cam sync if we have actually detected two groups and can get cam sync
          if (triggerInfo.toothSystemCount > 0 && triggerInfo.secondaryToothCount == (unsigned int)triggerInfo.toothAngles[triggerInfo.toothSystemCount+1]) { // Do a quick check if we already have cam sync
            triggerInfo.toothSystemCount++;
            if (triggerInfo.toothSystemCount > configPage2.nCylinders) { triggerInfo.toothSystemCount = 1; }
          }
          else { // Check for a pair of matching groups which tells us which group we are at, this should only happen when we don't have cam sync
            triggerInfo.toothSystemCount = 0; // We either haven't got cam sync yet or we lost cam sync
            for (byte group = 1; group <= configPage2.nCylinders; group++) {
              if (triggerInfo.secondaryToothCount == (unsigned int)triggerInfo.toothAngles[group] && secondaryToothLastCount == (byte)triggerInfo.toothAngles[group-1] ) { // Find a matching pattern/position
                triggerInfo.toothSystemCount = group;
                break;
              }
            }
          }
        }

        secondaryToothLastCount = triggerInfo.secondaryToothCount;
        //This is the first tooth in this group
        triggerInfo.secondaryToothCount = 1;

        triggerInfo.triggerSecFilterTime = 0; //This is used to prevent a condition where serious intermittent signals (Eg someone furiously plugging the sensor wire in and out) can leave the filter in an unrecoverable state

      }
      else if (triggerInfo.secondaryToothCount > 0) {
        //Normal tooth
        triggerInfo.secondaryToothCount++;
        triggerInfo.triggerSecFilterTime = triggerInfo.curGap2 >> 2; //Set filter at 25% of the current speed
      }
    }

    triggerInfo.toothLastSecToothTime = triggerInfo.curTime2;
  }
}

uint16_t getRPM_NGC(void)
{
  uint16_t tempRPM = 0;
  if( currentStatus.RPM < currentStatus.crankRPM)
  {
    if (BIT_CHECK(triggerInfo.decoderState, BIT_DECODER_TOOTH_ANG_CORRECT)) { tempRPM = crankingGetRPM(36, CRANK_SPEED); }
    else { tempRPM = currentStatus.RPM; } //Can't do per tooth RPM if we're at any of the missing teeth as it messes the calculation
  }
  else
  {
    tempRPM = stdGetRPM(CRANK_SPEED);
  }
  return tempRPM;
}

static inline uint16_t calcSetEndTeeth_NGC_SkipMissing(uint16_t toothNum) {
  if(toothNum == 17U || toothNum == 18U) { return 16U; } // These are missing teeth, so set the next one before instead
  if(toothNum == 35U || toothNum == 36U) { return 34U; } // These are missing teeth, so set the next one before instead
  if(toothNum == 53U || toothNum == 54U) { return 52U; } // These are missing teeth, so set the next one before instead
  if(toothNum > 70U) { return 70U; } // These are missing teeth, so set the next one before instead
  return toothNum;

}

static uint16_t __attribute__((noinline)) calcSetEndTeeth_NGC(int ignitionAngle, uint8_t toothAdder) {
  int16_t tempEndTooth = ignitionAngle - configPage4.triggerAngle;
#ifdef USE_LIBDIVIDE
  tempEndTooth = libdivide::libdivide_s16_do(tempEndTooth, &divtriggerInfo.triggerToothAngle);
#else
  tempEndTooth = tempEndTooth / (int16_t)triggerInfo.triggerToothAngle;
#endif
  return calcSetEndTeeth_NGC_SkipMissing(clampToToothCount(tempEndTooth - 1, toothAdder));
}

void triggerSetEndTeeth_NGC(void)
{
  byte toothAdder = 0;
  if( (configPage4.sparkMode == IGN_MODE_SEQUENTIAL) && (configPage4.TrigSpeed == CRANK_SPEED) ) { toothAdder = configPage4.triggerTeeth; }

  triggerInfo.ignition1EndTooth = calcSetEndTeeth_NGC(ignition1EndAngle, toothAdder);
  triggerInfo.ignition2EndTooth = calcSetEndTeeth_NGC(ignition2EndAngle, toothAdder);
  triggerInfo.ignition3EndTooth = calcSetEndTeeth_NGC(ignition3EndAngle, toothAdder);
  triggerInfo.ignition4EndTooth = calcSetEndTeeth_NGC(ignition4EndAngle, toothAdder);
  #if IGN_CHANNELS >= 6
  triggerInfo.ignition5EndTooth = calcSetEndTeeth_NGC(ignition5EndAngle, toothAdder);
  triggerInfo.ignition6EndTooth = calcSetEndTeeth_NGC(ignition6EndAngle, toothAdder);
  #endif

  #if IGN_CHANNELS >= 8
  triggerInfo.ignition7EndTooth = calcSetEndTeeth_NGC(ignition7EndAngle, toothAdder);
  triggerInfo.ignition8EndTooth = calcSetEndTeeth_NGC(ignition8EndAngle, toothAdder);
  #endif
}


