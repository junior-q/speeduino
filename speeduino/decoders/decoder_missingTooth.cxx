


/** A (single) multi-tooth wheel with one of more 'missing' teeth.
* The first tooth after the missing one is considered number 1 and is the basis for the trigger angle.
* Optionally a cam signal can be added to provide a sequential reference.
* @defgroup dec_miss Missing tooth wheel
* @{
*/
void triggerSetup_missingTooth(void)
{
  BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_IS_SEQUENTIAL);
  triggerInfo.triggerToothAngle = 360 / configPage4.triggerTeeth; //The number of degrees that passes from tooth to tooth
  if(configPage4.TrigSpeed == CAM_SPEED)
  {
    //Account for cam speed missing tooth
    triggerInfo.triggerToothAngle = 720 / configPage4.triggerTeeth;
    BIT_SET(triggerInfo.decoderState, BIT_DECODER_IS_SEQUENTIAL);
  }
  triggerInfo.triggerActualTeeth = configPage4.triggerTeeth - configPage4.triggerMissingTeeth; //The number of physical teeth on the wheel. Doing this here saves us a calculation each time in the interrupt
  triggerInfo.triggerFilterTime = (MICROS_PER_SEC / (MAX_RPM / 60U * configPage4.triggerTeeth)); //Trigger filter time is the shortest possible time (in uS) that there can be between crank teeth (ie at max RPM). Any pulses that occur faster than this time will be discarded as noise
  if (configPage4.trigPatternSec == SEC_TRIGGER_4_1)
  {
    triggerInfo.triggerSecFilterTime = MICROS_PER_MIN / MAX_RPM / 4U / 2U;
  }
  else
  {
    triggerInfo.triggerSecFilterTime = (MICROS_PER_SEC / (MAX_RPM / 60U));
  }
  BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_2ND_DERIV);
  triggerInfo.checkSyncToothCount = (configPage4.triggerTeeth) >> 1; //50% of the total teeth.
  triggerInfo.toothLastMinusOneToothTime = 0;
  triggerInfo.toothCurrentCount = 0;
  triggerInfo.secondaryToothCount = 0;
  triggerInfo.thirdToothCount = 0;
  triggerInfo.toothOneTime = 0;
  triggerInfo.toothOneMinusOneTime = 0;
  MAX_STALL_TIME = ((MICROS_PER_DEG_1_RPM/50U) * triggerInfo.triggerToothAngle * (configPage4.triggerMissingTeeth + 1U)); //Minimum 50rpm. (3333uS is the time per degree at 50rpm)

  if( (configPage4.TrigSpeed == CRANK_SPEED) && ( (configPage4.sparkMode == IGN_MODE_SEQUENTIAL) || (configPage2.injLayout == INJ_SEQUENTIAL) || (configPage6.vvtEnabled > 0)) ) { BIT_SET(triggerInfo.decoderState, BIT_DECODER_HAS_SECONDARY); }
  else { BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_HAS_SECONDARY); }
#ifdef USE_LIBDIVIDE
  divtriggerInfo.triggerToothAngle = libdivide::libdivide_s16_gen(triggerInfo.triggerToothAngle);
#endif
}

void triggerPri_missingTooth(void)
{
   triggerInfo.curTime = micros();
   triggerInfo.curGap = triggerInfo.curTime - triggerInfo.toothLastToothTime;
   if ( triggerInfo.curGap >= triggerInfo.triggerFilterTime ) //Pulses should never be less than triggerInfo.triggerFilterTime, so if they are it means a false trigger. (A 36-1 wheel at 8000pm will have triggers approx. every 200uS)
   {
     triggerInfo.toothCurrentCount++; //Increment the tooth counter
     BIT_SET(triggerInfo.decoderState, BIT_DECODER_VALID_TRIGGER); //Flag this pulse as being a valid trigger (ie that it passed filters)

     //if(triggerInfo.toothCurrentCount > triggerInfo.checkSyncToothCount || currentStatus.hasSync == false)
      if( (triggerInfo.toothLastToothTime > 0) && (triggerInfo.toothLastMinusOneToothTime > 0) )
      {
        bool isMissingTooth = false;

        /*
        Performance Optimisation:
        Only need to try and detect the missing tooth if:
        1. WE don't have sync yet
        2. We have sync and are in the final 1/4 of the wheel (Missing tooth will/should never occur in the first 3/4)
        3. RPM is under 2000. This is to ensure that we don't interfere with strange timing when cranking or idling. Optimisation not really required at these speeds anyway
        */
        if( (currentStatus.hasSync == false) || (currentStatus.RPM < 2000) || (triggerInfo.toothCurrentCount >= (3 * triggerInfo.triggerActualTeeth >> 2)) )
        {
          //Begin the missing tooth detection
          //If the time between the current tooth and the last is greater than 1.5x the time between the last tooth and the tooth before that, we make the assertion that we must be at the first tooth after the gap
          if(configPage4.triggerMissingTeeth == 1) { triggerInfo.targetGap = (3 * (triggerInfo.toothLastToothTime - triggerInfo.toothLastMinusOneToothTime)) >> 1; } //Multiply by 1.5 (Checks for a gap 1.5x greater than the last one) (Uses bitshift to multiply by 3 then divide by 2. Much faster than multiplying by 1.5)
          else { triggerInfo.targetGap = ((triggerInfo.toothLastToothTime - triggerInfo.toothLastMinusOneToothTime)) * configPage4.triggerMissingTeeth; } //Multiply by 2 (Checks for a gap 2x greater than the last one)

          if( (triggerInfo.toothLastToothTime == 0) || (triggerInfo.toothLastMinusOneToothTime == 0) ) { triggerInfo.curGap = 0; }

          if ( (triggerInfo.curGap > triggerInfo.targetGap) || (triggerInfo.toothCurrentCount > triggerInfo.triggerActualTeeth) )
          {
            //Missing tooth detected
            isMissingTooth = true;
            if( (triggerInfo.toothCurrentCount < triggerInfo.triggerActualTeeth) && (currentStatus.hasSync == true) )
            {
                //This occurs when we're at tooth #1, but haven't seen all the other teeth. This indicates a signal issue so we flag lost sync so this will attempt to resync on the next revolution.
                currentStatus.hasSync = false;
                BIT_CLEAR(currentStatus.status3, BIT_STATUS3_HALFSYNC); //No sync at all, so also clear HalfSync bit.
                currentStatus.syncLossCounter++;
            }
            //This is to handle a special case on startup where sync can be obtained and the system immediately thinks the revs have jumped:
            //else if (currentStatus.hasSync == false && triggerInfo.toothCurrentCount < triggerInfo.checkSyncToothCount ) { triggerInfo.triggerFilterTime = 0; }
            else
            {
                if((currentStatus.hasSync == true) || BIT_CHECK(currentStatus.status3, BIT_STATUS3_HALFSYNC))
                {
                  currentStatus.startRevolutions++; //Counter
                  if ( configPage4.TrigSpeed == CAM_SPEED ) { currentStatus.startRevolutions++; } //Add an extra revolution count if we're running at cam speed
                }
                else { currentStatus.startRevolutions = 0; }

                triggerInfo.toothCurrentCount = 1;
                if (configPage4.trigPatternSec == SEC_TRIGGER_POLL) // at tooth one check if the cam sensor is high or low in poll level mode
                {
                  if (configPage4.PollLevelPolarity == READ_SEC_TRIGGER()) { triggerInfo.revolutionOne = 1; }
                  else { triggerInfo.revolutionOne = 0; }
                }
                else {triggerInfo.revolutionOne = !triggerInfo.revolutionOne;} //Flip sequential revolution tracker if poll level is not used
                triggerInfo.toothOneMinusOneTime = triggerInfo.toothOneTime;
                triggerInfo.toothOneTime = triggerInfo.curTime;

                //if Sequential fuel or ignition is in use, further checks are needed before determining sync
                if( (configPage4.sparkMode == IGN_MODE_SEQUENTIAL) || (configPage2.injLayout == INJ_SEQUENTIAL) )
                {
                  //If either fuel or ignition is sequential, only declare sync if the cam tooth has been seen OR if the missing wheel is on the cam
                  if( (triggerInfo.secondaryToothCount > 0) || (configPage4.TrigSpeed == CAM_SPEED) || (configPage4.trigPatternSec == SEC_TRIGGER_POLL) || (configPage2.strokes == TWO_STROKE) )
                  {
                    currentStatus.hasSync = true;
                    BIT_CLEAR(currentStatus.status3, BIT_STATUS3_HALFSYNC); //the engine is fully synced so clear the Half Sync bit
                  }
                  else if(currentStatus.hasSync != true) { BIT_SET(currentStatus.status3, BIT_STATUS3_HALFSYNC); } //If there is primary trigger but no secondary we only have half sync.
                }
                else { currentStatus.hasSync = true;  BIT_CLEAR(currentStatus.status3, BIT_STATUS3_HALFSYNC); } //If nothing is using sequential, we have sync and also clear half sync bit
                if(configPage4.trigPatternSec == SEC_TRIGGER_SINGLE || configPage4.trigPatternSec == SEC_TRIGGER_TOYOTA_3) //Reset the secondary tooth counter to prevent it overflowing, done outside of sequental as v6 & v8 engines could be batch firing with VVT that needs the cam resetting
                {
                  triggerInfo.secondaryToothCount = 0;
                }

                triggerInfo.triggerFilterTime = 0; //This is used to prevent a condition where serious intermittent signals (Eg someone furiously plugging the sensor wire in and out) can leave the filter in an unrecoverable state
                triggerInfo.toothLastMinusOneToothTime = triggerInfo.toothLastToothTime;
                triggerInfo.toothLastToothTime = triggerInfo.curTime;
                BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_TOOTH_ANG_CORRECT); //The tooth angle is double at this point
            }
          }
        }

        if(isMissingTooth == false)
        {
          //Regular (non-missing) tooth
          setFilter(triggerInfo.curGap);
          triggerInfo.toothLastMinusOneToothTime = triggerInfo.toothLastToothTime;
          triggerInfo.toothLastToothTime = triggerInfo.curTime;
          BIT_SET(triggerInfo.decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);
        }
      }
      else
      {
        //We fall here on initial startup when enough teeth have not yet been seen
        triggerInfo.toothLastMinusOneToothTime = triggerInfo.toothLastToothTime;
        triggerInfo.toothLastToothTime = triggerInfo.curTime;
      }


      //NEW IGNITION MODE
      if( (configPage2.perToothIgn == true) && (!BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK)) )
      {
        int16_t crankAngle = ( (triggerInfo.toothCurrentCount-1) * triggerInfo.triggerToothAngle ) + configPage4.triggerAngle;
        if( (configPage4.sparkMode == IGN_MODE_SEQUENTIAL) && (triggerInfo.revolutionOne == true) && (configPage4.TrigSpeed == CRANK_SPEED) && (configPage2.strokes == FOUR_STROKE) )
        {
          crankAngle += 360;
          crankAngle = ignitionLimits(crankAngle);
          checkPerToothTiming(crankAngle, (configPage4.triggerTeeth + triggerInfo.toothCurrentCount));
        }
        else{ crankAngle = ignitionLimits(crankAngle); checkPerToothTiming(crankAngle, triggerInfo.toothCurrentCount); }
      }
   }
}

void triggerSec_missingTooth(void)
{
  triggerInfo.curTime2 = micros();
  triggerInfo.curGap2 = triggerInfo.curTime2 - triggerInfo.toothLastSecToothTime;

  //Safety check for initial startup
  if( (triggerInfo.toothLastSecToothTime == 0) )
  {
    triggerInfo.curGap2 = 0;
    triggerInfo.toothLastSecToothTime = triggerInfo.curTime2;
  }

  if ( triggerInfo.curGap2 >= triggerInfo.triggerSecFilterTime )
  {
    switch (configPage4.trigPatternSec)
    {
      case SEC_TRIGGER_4_1:
        triggerInfo.targetGap2 = (3 * (triggerInfo.toothLastSecToothTime - triggerInfo.toothLastMinusOneSecToothTime)) >> 1; //If the time between the current tooth and the last is greater than 1.5x the time between the last tooth and the tooth before that, we make the assertion that we must be at the first tooth after the gap
        triggerInfo.toothLastMinusOneSecToothTime = triggerInfo.toothLastSecToothTime;
        if ( (triggerInfo.curGap2 >= triggerInfo.targetGap2) || (triggerInfo.secondaryToothCount > 3) )
        {
          triggerInfo.secondaryToothCount = 1;
          triggerInfo.revolutionOne = 1; //Sequential revolution reset
          triggerInfo.triggerSecFilterTime = 0; //This is used to prevent a condition where serious intermittent signals (Eg someone furiously plugging the sensor wire in and out) can leave the filter in an unrecoverable state
          triggerRecordVVT1Angle();
        }
        else
        {
          triggerInfo.triggerSecFilterTime = triggerInfo.curGap2 >> 2; //Set filter at 25% of the current speed. Filter can only be recalc'd for the regular teeth, not the missing one.
          triggerInfo.secondaryToothCount++;
        }
        break;

      case SEC_TRIGGER_POLL:
        //Poll is effectively the same as SEC_TRIGGER_SINGLE, however we do not reset triggerInfo.revolutionOne
        //We do still need to record the angle for VVT though
        triggerInfo.triggerSecFilterTime = triggerInfo.curGap2 >> 1; //Next secondary filter is half the current gap
        triggerRecordVVT1Angle();
        break;

      case SEC_TRIGGER_SINGLE:
        //Standard single tooth cam trigger
        triggerInfo.revolutionOne = 1; //Sequential revolution reset
        triggerInfo.triggerSecFilterTime = triggerInfo.curGap2 >> 1; //Next secondary filter is half the current gap
        triggerInfo.secondaryToothCount++;
        triggerRecordVVT1Angle();
        break;

      case SEC_TRIGGER_TOYOTA_3:
        // designed for Toyota VVTI (2JZ) engine - 3 triggers on the cam.
        // the 2 teeth for this are within 1 rotation (1 tooth first 360, 2 teeth second 360)
        triggerInfo.secondaryToothCount++;
        if(triggerInfo.secondaryToothCount == 2)
        {
          triggerInfo.revolutionOne = 1; // sequential revolution reset
          triggerRecordVVT1Angle();
        }
        //Next secondary filter is 25% the current gap, done here so we don't get a great big gap for the 1st tooth
        triggerInfo.triggerSecFilterTime = triggerInfo.curGap2 >> 2;
        break;
    }
    triggerInfo.toothLastSecToothTime = triggerInfo.curTime2;
  } //Trigger filter
}

static inline void triggerRecordVVT1Angle (void)
{
  //Record the VVT Angle
  if( (configPage6.vvtEnabled > 0) && (triggerInfo.revolutionOne == 1) )
  {
    int16_t curAngle;
    curAngle = getCrankAngle();
    while(curAngle > 360) { curAngle -= 360; }
    curAngle -= configPage4.triggerAngle; //Value at TDC
    if( configPage6.vvtMode == VVT_MODE_CLOSED_LOOP ) { curAngle -= configPage10.vvtCL0DutyAng; }

    currentStatus.vvt1Angle = LOW_PASS_FILTER( (curAngle << 1), configPage4.ANGLEFILTER_VVT, currentStatus.vvt1Angle);
  }
}


void triggerThird_missingTooth(void)
{
//Record the VVT2 Angle (the only purpose of the third trigger)
//NB no filtering of this signal with current implementation unlike Cam (VVT1)

  int16_t curAngle;
  triggerInfo.curTime3 = micros();
  triggerInfo.curGap3 = triggerInfo.curTime3 - triggerInfo.toothLastThirdToothTime;

  //Safety check for initial startup
  if( (triggerInfo.toothLastThirdToothTime == 0) )
  {
    triggerInfo.curGap3 = 0;
    triggerInfo.toothLastThirdToothTime = triggerInfo.curTime3;
  }

  if ( triggerInfo.curGap3 >= triggerInfo.triggerThirdFilterTime )
  {
    triggerInfo.thirdToothCount++;
    triggerInfo.triggerThirdFilterTime = triggerInfo.curGap3 >> 2; //Next third filter is 25% the current gap

    curAngle = getCrankAngle();
    while(curAngle > 360) { curAngle -= 360; }
    curAngle -= configPage4.triggerAngle; //Value at TDC
    if( configPage6.vvtMode == VVT_MODE_CLOSED_LOOP ) { curAngle -= configPage4.vvt2CL0DutyAng; }
    //currentStatus.vvt2Angle = int8_t (curAngle); //vvt1Angle is only int8, but +/-127 degrees is enough for VVT control
    currentStatus.vvt2Angle = LOW_PASS_FILTER( (curAngle << 1), configPage4.ANGLEFILTER_VVT, currentStatus.vvt2Angle);

    triggerInfo.toothLastThirdToothTime = triggerInfo.curTime3;
  } //Trigger filter
}

uint16_t getRPM_missingTooth(void)
{
  uint16_t tempRPM = 0;
  if( currentStatus.RPM < currentStatus.crankRPM )
  {
    if(triggerInfo.toothCurrentCount != 1)
    {
      tempRPM = crankingGetRPM(configPage4.triggerTeeth, configPage4.TrigSpeed==CAM_SPEED); //Account for cam speed
    }
    else { tempRPM = currentStatus.RPM; } //Can't do per tooth RPM if we're at tooth #1 as the missing tooth messes the calculation
  }
  else
  {
    tempRPM = stdGetRPM(configPage4.TrigSpeed==CAM_SPEED); //Account for cam speed
  }
  return tempRPM;
}

int getCrankAngle_missingTooth(void)
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


static uint16_t __attribute__((noinline)) calcEndTeeth_missingTooth(int endAngle, uint8_t toothAdder) {
  //Temp variable used here to avoid potential issues if a trigger interrupt occurs part way through this function
  int16_t tempEndTooth;
#ifdef USE_LIBDIVIDE
  tempEndTooth = libdivide::libdivide_s16_do(endAngle - configPage4.triggerAngle, &divtriggerInfo.triggerToothAngle);
#else
  tempEndTooth = (endAngle - (int16_t)configPage4.triggerAngle) / (int16_t)triggerInfo.triggerToothAngle;
#endif
  //For higher tooth count triggers, add a 1 tooth margin to allow for calculation time.
  if(configPage4.triggerTeeth > 12U) { tempEndTooth = tempEndTooth - 1; }

  // Clamp to tooth count
  return clampToActualTeeth(clampToToothCount(tempEndTooth, toothAdder), toothAdder);
}

void triggerSetEndTeeth_missingTooth(void)
{
  uint8_t toothAdder = 0;
  if( ((configPage4.sparkMode == IGN_MODE_SEQUENTIAL) || (configPage4.sparkMode == IGN_MODE_SINGLE)) && (configPage4.TrigSpeed == CRANK_SPEED) && (configPage2.strokes == FOUR_STROKE) ) { toothAdder = configPage4.triggerTeeth; }

  triggerInfo.ignition1EndTooth = calcEndTeeth_missingTooth(ignition1EndAngle, toothAdder);
  triggerInfo.ignition2EndTooth = calcEndTeeth_missingTooth(ignition2EndAngle, toothAdder);
  triggerInfo.ignition3EndTooth = calcEndTeeth_missingTooth(ignition3EndAngle, toothAdder);
  triggerInfo.ignition4EndTooth = calcEndTeeth_missingTooth(ignition4EndAngle, toothAdder);
#if IGN_CHANNELS >= 5
  triggerInfo.ignition5EndTooth = calcEndTeeth_missingTooth(ignition5EndAngle, toothAdder);
#endif
#if IGN_CHANNELS >= 6
  triggerInfo.ignition6EndTooth = calcEndTeeth_missingTooth(ignition6EndAngle, toothAdder);
#endif
#if IGN_CHANNELS >= 7
  triggerInfo.ignition7EndTooth = calcEndTeeth_missingTooth(ignition7EndAngle, toothAdder);
#endif
#if IGN_CHANNELS >= 8
  triggerInfo.ignition8EndTooth = calcEndTeeth_missingTooth(ignition8EndAngle, toothAdder);
#endif
}
/** @} */


