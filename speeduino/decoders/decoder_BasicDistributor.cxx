

/** Basic Distributor where tooth count is equal to the number of cylinders and teeth are evenly spaced on the cam.
* No position sensing (Distributor is retained) so crank angle is
* a made up figure based purely on the first teeth to be seen.
* Note: This is a very simple decoder. See http://www.megamanual.com/ms2/GM_7pinHEI.htm
* @defgroup dec_dist Basic Distributor
* @{
*/
void triggerSetup_BasicDistributor(void)
{
  triggerInfo.triggerActualTeeth = configPage2.nCylinders;
  if(triggerInfo.triggerActualTeeth == 0) { triggerInfo.triggerActualTeeth = 1; }

  //The number of degrees that passes from tooth to tooth. Depends on number of cylinders and whether 4 or 2 stroke
  if(configPage2.strokes == FOUR_STROKE) { triggerInfo.triggerToothAngle = 720U / triggerInfo.triggerActualTeeth; }
  else { triggerInfo.triggerToothAngle = 360U / triggerInfo.triggerActualTeeth; }

  triggerInfo.triggerFilterTime = MICROS_PER_MIN / MAX_RPM / configPage2.nCylinders; // Minimum time required between teeth
  triggerInfo.triggerFilterTime = triggerInfo.triggerFilterTime / 2; //Safety margin
  triggerInfo.triggerFilterTime = 0;
  BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_2ND_DERIV);
  BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_IS_SEQUENTIAL);
  BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_HAS_SECONDARY);
  triggerInfo.toothCurrentCount = 0; //Default value
  BIT_SET(triggerInfo.decoderState, BIT_DECODER_HAS_FIXED_CRANKING);
  BIT_SET(triggerInfo.decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);
  if(configPage2.nCylinders <= 4U) { MAX_STALL_TIME = ((MICROS_PER_DEG_1_RPM/90U) * triggerInfo.triggerToothAngle); }//Minimum 90rpm. (1851uS is the time per degree at 90rpm). This uses 90rpm rather than 50rpm due to the potentially very high stall time on a 4 cylinder if we wait that long.
  else { MAX_STALL_TIME = ((MICROS_PER_DEG_1_RPM/50U) * triggerInfo.triggerToothAngle); } //Minimum 50rpm. (3200uS is the time per degree at 50rpm).

}

void triggerPri_BasicDistributor(void)
{
  triggerInfo.curTime = micros();
  triggerInfo.curGap = triggerInfo.curTime - triggerInfo.toothLastToothTime;
  if ( (triggerInfo.curGap >= triggerInfo.triggerFilterTime) )
  {
    if(currentStatus.hasSync == true) { setFilter(triggerInfo.curGap); } //Recalc the new filter value
    else { triggerInfo.triggerFilterTime = 0; } //If we don't yet have sync, ensure that the filter won't prevent future valid pulses from being ignored.

    if( (triggerInfo.toothCurrentCount == triggerInfo.triggerActualTeeth) || (currentStatus.hasSync == false) ) //Check if we're back to the beginning of a revolution
    {
      triggerInfo.toothCurrentCount = 1; //Reset the counter
      triggerInfo.toothOneMinusOneTime = triggerInfo.toothOneTime;
      triggerInfo.toothOneTime = triggerInfo.curTime;
      currentStatus.hasSync = true;
      currentStatus.startRevolutions++; //Counter
    }
    else
    {
      if( (triggerInfo.toothCurrentCount < triggerInfo.triggerActualTeeth) ) { triggerInfo.toothCurrentCount++; } //Increment the tooth counter
      else
      {
        //This means triggerInfo.toothCurrentCount is greater than triggerInfo.triggerActualTeeth, which is bad.
        //If we have sync here then there's a problem. Throw a sync loss
        if( currentStatus.hasSync == true )
        {
          currentStatus.syncLossCounter++;
          currentStatus.hasSync = false;
        }
      }

    }

    BIT_SET(triggerInfo.decoderState, BIT_DECODER_VALID_TRIGGER); //Flag this pulse as being a valid trigger (ie that it passed filters)

    if ( configPage4.ignCranklock && BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) )
    {
      endCoil1Charge();
      endCoil2Charge();
      endCoil3Charge();
      endCoil4Charge();
    }

    if(configPage2.perToothIgn == true)
    {
      int16_t crankAngle = ( (triggerInfo.toothCurrentCount-1) * triggerInfo.triggerToothAngle ) + configPage4.triggerAngle;
      crankAngle = ignitionLimits((crankAngle));
      uint16_t currentTooth = triggerInfo.toothCurrentCount;
      if(triggerInfo.toothCurrentCount > (triggerInfo.triggerActualTeeth/2) ) { currentTooth = (triggerInfo.toothCurrentCount - (triggerInfo.triggerActualTeeth/2)); }
      checkPerToothTiming(crankAngle, currentTooth);
    }

    triggerInfo.toothLastMinusOneToothTime = triggerInfo.toothLastToothTime;
    triggerInfo.toothLastToothTime = triggerInfo.curTime;
  } //Trigger filter
}
void triggerSec_BasicDistributor(void) { return; } //Not required
uint16_t getRPM_BasicDistributor(void)
{
  uint16_t tempRPM;
  uint8_t distributorSpeed = CAM_SPEED; //Default to cam speed
  if(configPage2.strokes == TWO_STROKE) { distributorSpeed = CRANK_SPEED; } //For 2 stroke distributors, the tooth rate is based on crank speed, not 'cam'

  if( currentStatus.RPM < currentStatus.crankRPM || currentStatus.RPM < 1500)
  {
    tempRPM = crankingGetRPM(triggerInfo.triggerActualTeeth, distributorSpeed);
  }
  else { tempRPM = stdGetRPM(distributorSpeed); }

  MAX_STALL_TIME = revolutionTime << 1; //Set the stall time to be twice the current RPM. This is a safe figure as there should be no single revolution where this changes more than this
  if(triggerInfo.triggerActualTeeth == 1) { MAX_STALL_TIME = revolutionTime << 1; } //Special case for 1 cylinder engines that only get 1 pulse every 720 degrees
  if(MAX_STALL_TIME < 366667UL) { MAX_STALL_TIME = 366667UL; } //Check for 50rpm minimum

  return tempRPM;

}


int getCrankAngle_BasicDistributor(void)
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

    int crankAngle = ((temptoothCurrentCount - 1) * triggerInfo.triggerToothAngle) + configPage4.triggerAngle; //Number of teeth that have passed since tooth 1, multiplied by the angle each tooth represents, plus the angle that tooth 1 is ATDC. This gives accuracy only to the nearest tooth.

    //Estimate the number of degrees travelled since the last tooth}
    triggerInfo.elapsedTime = (triggerInfo.lastCrankAngleCalc - temptoothLastToothTime);

    //crankAngle += timeToAngleDegPerMicroSec(triggerInfo.elapsedTime);
    crankAngle += timeToAngleIntervalTooth(triggerInfo.elapsedTime);


    if (crankAngle >= 720) { crankAngle -= 720; }
    if (crankAngle < 0) { crankAngle += CRANK_ANGLE_MAX; }

    return crankAngle;
}

void triggerSetEndTeeth_BasicDistributor(void)
{

  int tempEndAngle = (ignition1EndAngle - configPage4.triggerAngle);
  tempEndAngle = ignitionLimits((tempEndAngle));

  switch(configPage2.nCylinders)
  {
    case 4:
      if( (tempEndAngle > 180) || (tempEndAngle <= 0) )
      {
        triggerInfo.ignition1EndTooth = 2;
        triggerInfo.ignition2EndTooth = 1;
      }
      else
      {
        triggerInfo.ignition1EndTooth = 1;
        triggerInfo.ignition2EndTooth = 2;
      }
      break;
    case 3: //Shared with 6 cylinder
    case 6:
      if( (tempEndAngle > 120) && (tempEndAngle <= 240) )
      {
        triggerInfo.ignition1EndTooth = 2;
        triggerInfo.ignition2EndTooth = 3;
        triggerInfo.ignition3EndTooth = 1;
      }
      else if( (tempEndAngle > 240) || (tempEndAngle <= 0) )
      {
        triggerInfo.ignition1EndTooth = 3;
        triggerInfo.ignition2EndTooth = 1;
        triggerInfo.ignition3EndTooth = 2;
      }
      else
      {
        triggerInfo.ignition1EndTooth = 1;
        triggerInfo.ignition2EndTooth = 2;
        triggerInfo.ignition3EndTooth = 3;
      }
      break;
    case 8:
      if( (tempEndAngle > 90) && (tempEndAngle <= 180) )
      {
        triggerInfo.ignition1EndTooth = 2;
        triggerInfo.ignition2EndTooth = 3;
        triggerInfo.ignition3EndTooth = 4;
        triggerInfo.ignition4EndTooth = 1;
      }
      else if( (tempEndAngle > 180) && (tempEndAngle <= 270) )
      {
        triggerInfo.ignition1EndTooth = 3;
        triggerInfo.ignition2EndTooth = 4;
        triggerInfo.ignition3EndTooth = 1;
        triggerInfo.ignition4EndTooth = 2;
      }
      else if( (tempEndAngle > 270) || (tempEndAngle <= 0) )
      {
        triggerInfo.ignition1EndTooth = 4;
        triggerInfo.ignition2EndTooth = 1;
        triggerInfo.ignition3EndTooth = 2;
        triggerInfo.ignition4EndTooth = 3;
      }
      else
      {
        triggerInfo.ignition1EndTooth = 1;
        triggerInfo.ignition2EndTooth = 2;
        triggerInfo.ignition3EndTooth = 3;
        triggerInfo.ignition4EndTooth = 4;
      }
      break;
  }
}

