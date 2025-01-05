
/** Renix 44-2-2  and 66-2-2-2 decoder.
* Renix trigger wheel doesn't decode into 360 degrees nicely (360/44 = 8.18 degrees or 360/66 = 5.454545). Speeduino can't handle any teeth that have a decimal point.
* Solution is to count teeth, every 11 teeth = a proper angle. For 66 tooth decoder its 60 degrees per 11 teeth, for 44 tooth decoder its 90 degrees per 11 teeth.
* This means the system sees 4 teeth on the 44 tooth wheel and 6 teeth on the 66 tooth wheel.
* Double missing tooth in the pattern is actually a large tooth and a large gap. If the trigger is set to rising you'll see the start of the large tooth
* then the gap. If its not set to rising the code won't work due to seeing two gaps
*
*
* @defgroup dec_renix Renix decoder
* @{
*/
void triggerSetup_Renix(void)
{
  if( configPage2.nCylinders == 4)
  {
    triggerInfo.triggerToothAngle = 90; //The number of degrees that passes from tooth to tooth (primary) this changes between 41 and 49 degrees
    configPage4.triggerTeeth = 4; // wheel has 44 teeth but we use these to work out which tooth angle to use, therefore speeduino thinks we only have 8 teeth.
    configPage4.triggerMissingTeeth = 0;
    triggerInfo.triggerActualTeeth = 4; //The number of teeth we're pretending physically existing on the wheel.
    triggerInfo.triggerFilterTime = (MICROS_PER_SEC / (MAX_RPM / 60U * 44U)); //Trigger filter time is the shortest possible time (in uS) that there can be between crank teeth (ie at max RPM). Any pulses that occur faster than this time will be discarded as noise
  }
  else if (configPage2.nCylinders == 6)
  {
    triggerInfo.triggerToothAngle = 60;
    configPage4.triggerTeeth = 6; // wheel has 44 teeth but we use these to work out which tooth angle to use, therefore speeduino thinks we only have 6 teeth.
    configPage4.triggerMissingTeeth = 0;
    triggerInfo.triggerActualTeeth = 6; //The number of teeth we're pretending physically existing on the wheel.
    triggerInfo.triggerFilterTime = (MICROS_PER_SEC / (MAX_RPM / 60U * 66U)); //Trigger filter time is the shortest possible time (in uS) that there can be between crank teeth (ie at max RPM). Any pulses that occur faster than this time will be discarded as noise
  }

  MAX_STALL_TIME = ((MICROS_PER_DEG_1_RPM/50U) * triggerInfo.triggerToothAngle); //Minimum 50rpm. (3333uS is the time per degree at 50rpm). Largest gap between teeth is 90 or 60 degrees depending on decoder.
  BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_HAS_SECONDARY);

  triggerInfo.toothSystemCount = 1;
  triggerInfo.toothCurrentCount = 1;
  triggerInfo.toothLastToothTime = 0;
#ifdef USE_LIBDIVIDE
  divtriggerInfo.triggerToothAngle = libdivide::libdivide_s16_gen(triggerInfo.triggerToothAngle);
#endif
}


// variables used to help calculate gap on the physical 44 or 66 teeth we're pretending don't exist in most of the speeduino code
// reusing existing variables to save storage space as these aren't used in the code for their original purpose.
#define renixSystemLastToothTime         triggerInfo.toothLastToothRisingTime
#define renixSystemLastMinusOneToothTime triggerInfo.toothLastSecToothRisingTime

void triggerPri_Renix(void)
{
  triggerInfo.curTime = micros();
  triggerInfo.curGap = triggerInfo.curTime - renixSystemLastToothTime;

  if ( triggerInfo.curGap >= triggerInfo.triggerFilterTime )
  {

    triggerInfo.toothSystemCount++;

    if( renixSystemLastToothTime != 0 && renixSystemLastMinusOneToothTime != 0)
    { triggerInfo.targetGap = (2 * (renixSystemLastToothTime - renixSystemLastMinusOneToothTime));}  // in real world the physical 2 tooth gap is bigger than 2 teeth - more like 2.5
    else
    { triggerInfo.targetGap = 100000000L; } // random large number to stop system thinking we have a gap for the first few teeth on start up

    if( triggerInfo.curGap >= triggerInfo.targetGap )
    {
      /* add two teeth to account for the gap we've just seen */
      triggerInfo.toothSystemCount++;
      triggerInfo.toothSystemCount++;

      if( triggerInfo.toothSystemCount != 12) // if not 12 (the first tooth after the gap) then we've lost sync
      {
        // lost sync
        currentStatus.hasSync = false;
        currentStatus.syncLossCounter++;
        triggerInfo.toothSystemCount = 1; // first tooth after gap is always 1
        triggerInfo.toothCurrentCount = 1; // Reset as we've lost sync
      }
    }
    else
    {
      //Recalc the new filter value, only do this on the single gap tooth
      setFilter(triggerInfo.curGap);
    }
    renixSystemLastMinusOneToothTime = renixSystemLastToothTime; // needed for target gap calculation
    renixSystemLastToothTime = triggerInfo.curTime;

    if( triggerInfo.toothSystemCount == 12  || triggerInfo.toothLastToothTime == 0) // triggerInfo.toothLastToothTime used to ensure we set the value so the code that handles the fuel pump in speeduino.ini has a value to use once the engine is running.
    {
      triggerInfo.toothCurrentCount++;

      if( (configPage2.nCylinders == 6 && triggerInfo.toothCurrentCount == 7) ||    // 6 Pretend teeth on the 66 tooth wheel, if get to severn rotate round back to first tooth
          (configPage2.nCylinders == 4 && triggerInfo.toothCurrentCount == 5 ) )    // 4 Pretend teeth on the 44 tooth wheel, if get to five rotate round back to first tooth
      {
        triggerInfo.toothOneMinusOneTime = triggerInfo.toothOneTime;
        triggerInfo.toothOneTime = triggerInfo.curTime;
        currentStatus.hasSync = true;
        currentStatus.startRevolutions++; //Counter
        triggerInfo.revolutionOne = !triggerInfo.revolutionOne;
        triggerInfo.toothCurrentCount = 1;
      }

      triggerInfo.toothSystemCount = 1;
      triggerInfo.toothLastMinusOneToothTime = triggerInfo.toothLastToothTime;
      triggerInfo.toothLastToothTime = triggerInfo.curTime;


      //NEW IGNITION MODE
      if( (configPage2.perToothIgn == true) && (!BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK)) )
      {
        int16_t crankAngle = ( (triggerInfo.toothCurrentCount - 1) * triggerInfo.triggerToothAngle ) + configPage4.triggerAngle;
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
}

static uint16_t __attribute__((noinline)) calcEndTeeth_Renix(int ignitionAngle, uint8_t toothAdder) {
  int16_t tempEndTooth = ignitionAngle - configPage4.triggerAngle;
#ifdef USE_LIBDIVIDE
  tempEndTooth = libdivide::libdivide_s16_do(tempEndTooth, &divtriggerInfo.triggerToothAngle);
#else
  tempEndTooth = tempEndTooth / (int16_t)triggerInfo.triggerToothAngle;
#endif
  tempEndTooth = tempEndTooth - 1;
  // Clamp to tooth count
  return clampToActualTeeth(clampToToothCount(tempEndTooth, toothAdder), toothAdder);
}

void triggerSetEndTeeth_Renix(void)
{
  byte toothAdder = 0;
  if( (configPage4.sparkMode == IGN_MODE_SEQUENTIAL) && (configPage4.TrigSpeed == CRANK_SPEED) ) { toothAdder = configPage4.triggerTeeth; }

  //Temp variables are used here to avoid potential issues if a trigger interrupt occurs part way through this function

  triggerInfo.ignition1EndTooth = calcEndTeeth_Renix(ignition1EndAngle, toothAdder);
  triggerInfo.ignition2EndTooth = calcEndTeeth_Renix(ignition2EndAngle, toothAdder);
  currentStatus.canin[1] = triggerInfo.ignition2EndTooth;
  triggerInfo.ignition3EndTooth = calcEndTeeth_Renix(ignition3EndAngle, toothAdder);
  triggerInfo.ignition4EndTooth = calcEndTeeth_Renix(ignition4EndAngle, toothAdder);
#if IGN_CHANNELS >= 5
  triggerInfo.ignition5EndTooth = calcEndTeeth_Renix(ignition5EndAngle, toothAdder);
#endif
#if IGN_CHANNELS >= 6
  triggerInfo.ignition6EndTooth = calcEndTeeth_Renix(ignition6EndAngle, toothAdder);
#endif
#if IGN_CHANNELS >= 7
  triggerInfo.ignition7EndTooth = calcEndTeeth_Renix(ignition7EndAngle, toothAdder);
#endif
#if IGN_CHANNELS >= 8
  triggerInfo.ignition8EndTooth = calcEndTeeth_Renix(ignition8EndAngle, toothAdder);
#endif
}

/** @} */

