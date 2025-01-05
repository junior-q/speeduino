
/** Dual wheels - 2 wheels located either both on the crank or with the primary on the crank and the secondary on the cam.
Note: There can be no missing teeth on the primary wheel.
* @defgroup dec_dual Dual wheels
* @{
*/
/** Dual Wheel Setup.
 *
 * */
void triggerSetup_DualWheel(void)
{
  triggerInfo.triggerToothAngle = 360 / configPage4.triggerTeeth; //The number of degrees that passes from tooth to tooth

  if(configPage4.TrigSpeed == CAM_SPEED) { triggerInfo.triggerToothAngle = 720 / configPage4.triggerTeeth; } //Account for cam speed

  triggerInfo.toothCurrentCount = UINT8_MAX; //Default value
  triggerInfo.triggerFilterTime = (MICROS_PER_SEC / (MAX_RPM / 60U * configPage4.triggerTeeth)); //Trigger filter time is the shortest possible time (in uS) that there can be between crank teeth (ie at max RPM). Any pulses that occur faster than this time will be discarded as noise
  triggerInfo.triggerSecFilterTime = (MICROS_PER_SEC / (MAX_RPM / 60U * 2U)) / 2U; //Same as above, but fixed at 2 teeth on the secondary input and divided by 2 (for cam speed)

  BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_2ND_DERIV);
  BIT_SET(triggerInfo.decoderState, BIT_DECODER_IS_SEQUENTIAL);
  BIT_SET(triggerInfo.decoderState, BIT_DECODER_TOOTH_ANG_CORRECT); //This is always true for this pattern
  BIT_SET(triggerInfo.decoderState, BIT_DECODER_HAS_SECONDARY);
  MAX_STALL_TIME = ((MICROS_PER_DEG_1_RPM/50U) * triggerInfo.triggerToothAngle); //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
#ifdef USE_LIBDIVIDE
  divtriggerInfo.triggerToothAngle = libdivide::libdivide_s16_gen(triggerInfo.triggerToothAngle);
#endif
}

/** Dual Wheel Primary.
 *
 * */
void triggerPri_DualWheel(void)
{
    triggerInfo.curTime = micros();
    triggerInfo.curGap = triggerInfo.curTime - triggerInfo.toothLastToothTime;
    if ( triggerInfo.curGap >= triggerInfo.triggerFilterTime )
    {
      triggerInfo.toothCurrentCount++; //Increment the tooth counter
      BIT_SET(triggerInfo.decoderState, BIT_DECODER_VALID_TRIGGER); //Flag this pulse as being a valid trigger (ie that it passed filters)

      triggerInfo.toothLastMinusOneToothTime = triggerInfo.toothLastToothTime;
      triggerInfo.toothLastToothTime = triggerInfo.curTime;

      if ( currentStatus.hasSync == true )
      {
        if ( (triggerInfo.toothCurrentCount == 1) || (triggerInfo.toothCurrentCount > configPage4.triggerTeeth) )
        {
          triggerInfo.toothCurrentCount = 1;
          triggerInfo.revolutionOne = !triggerInfo.revolutionOne; //Flip sequential revolution tracker
          triggerInfo.toothOneMinusOneTime = triggerInfo.toothOneTime;
          triggerInfo.toothOneTime = triggerInfo.curTime;
          currentStatus.startRevolutions++; //Counter
          if ( configPage4.TrigSpeed == CAM_SPEED ) { currentStatus.startRevolutions++; } //Add an extra revolution count if we're running at cam speed
        }

        setFilter(triggerInfo.curGap); //Recalc the new filter value
      }

      //NEW IGNITION MODE
      if( (configPage2.perToothIgn == true) && (!BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK)) )
      {
        int16_t crankAngle = ( (triggerInfo.toothCurrentCount-1) * triggerInfo.triggerToothAngle ) + configPage4.triggerAngle;
        uint16_t currentTooth;
        if( (configPage4.sparkMode == IGN_MODE_SEQUENTIAL) && (triggerInfo.revolutionOne == true) && (configPage4.TrigSpeed == CRANK_SPEED) )
        {
          crankAngle += 360;
          currentTooth = (configPage4.triggerTeeth + triggerInfo.toothCurrentCount);
        }
        else{ currentTooth = triggerInfo.toothCurrentCount; }
        checkPerToothTiming(crankAngle, currentTooth);
      }
   } //Trigger filter
}
/** Dual Wheel Secondary.
 *
 * */
void triggerSec_DualWheel(void)
{
  triggerInfo.curTime2 = micros();
  triggerInfo.curGap2 = triggerInfo.curTime2 - triggerInfo.toothLastSecToothTime;
  if ( triggerInfo.curGap2 >= triggerInfo.triggerSecFilterTime )
  {
    triggerInfo.toothLastSecToothTime = triggerInfo.curTime2;
    triggerInfo.triggerSecFilterTime = triggerInfo.curGap2 >> 2; //Set filter at 25% of the current speed

    if( (currentStatus.hasSync == false) || (currentStatus.startRevolutions <= configPage4.StgCycles) )
    {
      triggerInfo.toothLastToothTime = micros();
      triggerInfo.toothLastMinusOneToothTime = micros() - ((MICROS_PER_MIN/10U) / configPage4.triggerTeeth); //Fixes RPM at 10rpm until a full revolution has taken place
      triggerInfo.toothCurrentCount = configPage4.triggerTeeth;
      triggerInfo.triggerFilterTime = 0; //Need to turn the filter off here otherwise the first primary tooth after achieving sync is ignored

      currentStatus.hasSync = true;
    }
    else
    {
      if ( (triggerInfo.toothCurrentCount != configPage4.triggerTeeth) && (currentStatus.startRevolutions > 2)) { currentStatus.syncLossCounter++; } //Indicates likely sync loss.
      if (configPage4.useResync == 1) { triggerInfo.toothCurrentCount = configPage4.triggerTeeth; }
    }

    triggerInfo.revolutionOne = 1; //Sequential revolution reset
  }
  else
  {
    triggerInfo.triggerSecFilterTime = revolutionTime >> 1; //Set filter at 25% of the current cam speed. This needs to be performed here to prevent a situation where the RPM and triggerInfo.triggerSecFilterTime get out of alignment and triggerInfo.curGap2 never exceeds the filter value
  } //Trigger filter
}
/** Dual Wheel - Get RPM.
 *
 * */
uint16_t getRPM_DualWheel(void)
{
  if( currentStatus.hasSync == true )
  {
    //Account for cam speed
    if( currentStatus.RPM < currentStatus.crankRPM )
    {
      return crankingGetRPM(configPage4.triggerTeeth, configPage4.TrigSpeed==CAM_SPEED);
    }
    else
    {
      return stdGetRPM(configPage4.TrigSpeed==CAM_SPEED);
    }
  }
  return 0U;
}

/** Dual Wheel - Get Crank angle.
 *
 * */
int getCrankAngle_DualWheel(void)
{
    //This is the current angle ATDC the engine is at. This is the last known position based on what tooth was last 'seen'. It is only accurate to the resolution of the trigger wheel (Eg 36-1 is 10 degrees)
    unsigned long temptoothLastToothTime;
    int temptoothCurrentCount;
    bool temprevolutionOne;
    //Grab some variables that are used in the trigger code and assign them to temp variables.
    noInterrupts();
    temptoothCurrentCount = triggerInfo.toothCurrentCount;
    temptoothLastToothTime = triggerInfo.toothLastToothTime;
    temprevolutionOne = triggerInfo.revolutionOne;
    triggerInfo.lastCrankAngleCalc = micros();
    interrupts();

    //Handle case where the secondary tooth was the last one seen
    if(temptoothCurrentCount == 0) { temptoothCurrentCount = configPage4.triggerTeeth; }

    int crankAngle = ((temptoothCurrentCount - 1) * triggerInfo.triggerToothAngle) + configPage4.triggerAngle; //Number of teeth that have passed since tooth 1, multiplied by the angle each tooth represents, plus the angle that tooth 1 is ATDC. This gives accuracy only to the nearest tooth.

    triggerInfo.elapsedTime = (triggerInfo.lastCrankAngleCalc - temptoothLastToothTime);
    crankAngle += timeToAngleDegPerMicroSec(triggerInfo.elapsedTime);

    //Sequential check (simply sets whether we're on the first or 2nd revolution of the cycle)
    if ( (temprevolutionOne == true) && (configPage4.TrigSpeed == CRANK_SPEED) ) { crankAngle += 360; }

    if (crankAngle >= 720) { crankAngle -= 720; }
    if (crankAngle < 0) { crankAngle += CRANK_ANGLE_MAX; }

    return crankAngle;
}

static uint16_t __attribute__((noinline)) calcEndTeeth_DualWheel(int ignitionAngle, uint8_t toothAdder) {
  int16_t tempEndTooth =
#ifdef USE_LIBDIVIDE
      libdivide::libdivide_s16_do(ignitionAngle - configPage4.triggerAngle, &divtriggerInfo.triggerToothAngle);
#else
      (ignitionAngle - (int16_t)configPage4.triggerAngle) / (int16_t)triggerInfo.triggerToothAngle;
#endif
  return clampToToothCount(tempEndTooth, toothAdder);
}

/** Dual Wheel - Set End Teeth.
 *
 * */
void triggerSetEndTeeth_DualWheel(void)
{
  //The toothAdder variable is used for when a setup is running sequentially, but the primary wheel is running at crank speed. This way the count of teeth will go up to 2* the number of primary teeth to allow for a sequential count.
  byte toothAdder = 0;
  if( (configPage4.sparkMode == IGN_MODE_SEQUENTIAL) && (configPage4.TrigSpeed == CRANK_SPEED) ) { toothAdder = configPage4.triggerTeeth; }

  triggerInfo.ignition1EndTooth = calcEndTeeth_DualWheel(ignition1EndAngle, toothAdder);
  triggerInfo.ignition2EndTooth = calcEndTeeth_DualWheel(ignition2EndAngle, toothAdder);
  triggerInfo.ignition3EndTooth = calcEndTeeth_DualWheel(ignition3EndAngle, toothAdder);
  triggerInfo.ignition4EndTooth = calcEndTeeth_DualWheel(ignition4EndAngle, toothAdder);
#if IGN_CHANNELS >= 5
  triggerInfo.ignition5EndTooth = calcEndTeeth_DualWheel(ignition5EndAngle, toothAdder);
#endif
#if IGN_CHANNELS >= 6
  triggerInfo.ignition6EndTooth = calcEndTeeth_DualWheel(ignition6EndAngle, toothAdder);
#endif
#if IGN_CHANNELS >= 7
  triggerInfo.ignition7EndTooth = calcEndTeeth_DualWheel(ignition7EndAngle, toothAdder);
#endif
#if IGN_CHANNELS >= 8
  triggerInfo.ignition8EndTooth = calcEndTeeth_DualWheel(ignition8EndAngle, toothAdder);
#endif
}
/** @} */
/** @} */

