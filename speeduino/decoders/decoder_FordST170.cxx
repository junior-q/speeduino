

/** Ford ST170 - a dedicated decoder for 01-04 Ford Focus ST170/SVT engine.
Standard 36-1 trigger wheel running at crank speed and 8-3 trigger wheel running at cam speed.
* @defgroup dec_ford_st170 Ford ST170 (01-04 Focus)
* @{
*/
void triggerSetup_FordST170(void)
{
  //Set these as we are using the existing missing tooth primary decoder and these will never change.
  configPage4.triggerTeeth = 36;
  configPage4.triggerMissingTeeth = 1;
  configPage4.TrigSpeed = CRANK_SPEED;

  triggerInfo.triggerToothAngle = 360 / configPage4.triggerTeeth; //The number of degrees that passes from tooth to tooth
  triggerInfo.triggerActualTeeth = configPage4.triggerTeeth - configPage4.triggerMissingTeeth; //The number of physical teeth on the wheel. Doing this here saves us a calculation each time in the interrupt
  triggerInfo.triggerFilterTime = (MICROS_PER_SEC / (MAX_RPM / 60U * configPage4.triggerTeeth)); //Trigger filter time is the shortest possible time (in uS) that there can be between crank teeth (ie at max RPM). Any pulses that occur faster than this time will be discarded as noise

  triggerInfo.triggerSecFilterTime = MICROS_PER_MIN / MAX_RPM / 8U / 2U; //Cam pattern is 8-3, so 2 nearest teeth are 90 deg crank angle apart. Cam can be advanced by 60 deg, so going from fully retarded to fully advanced closes the gap to 30 deg. Zetec cam pulleys aren't keyed from factory, so I subtracted additional 10 deg to avoid filter to be too aggressive. And there you have it 720/20=36.

  BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_2ND_DERIV);
  BIT_SET(triggerInfo.decoderState, BIT_DECODER_IS_SEQUENTIAL);
  BIT_SET(triggerInfo.decoderState, BIT_DECODER_HAS_SECONDARY);
  triggerInfo.checkSyncToothCount = (36) >> 1; //50% of the total teeth.
  triggerInfo.toothLastMinusOneToothTime = 0;
  triggerInfo.toothCurrentCount = 0;
  triggerInfo.secondaryToothCount = 0;
  triggerInfo.toothOneTime = 0;
  triggerInfo.toothOneMinusOneTime = 0;
  MAX_STALL_TIME = ((MICROS_PER_DEG_1_RPM/50U) * triggerInfo.triggerToothAngle * (1U + 1U)); //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
#ifdef USE_LIBDIVIDE
  divtriggerInfo.triggerToothAngle = libdivide::libdivide_s16_gen(triggerInfo.triggerToothAngle);
#endif
}

void triggerSec_FordST170(void)
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
      triggerInfo.targetGap2 = (3 * (triggerInfo.toothLastSecToothTime - triggerInfo.toothLastMinusOneSecToothTime)) >> 1; //If the time between the current tooth and the last is greater than 1.5x the time between the last tooth and the tooth before that, we make the assertion that we must be at the first tooth after the gap
      triggerInfo.toothLastMinusOneSecToothTime = triggerInfo.toothLastSecToothTime;
      if ( (triggerInfo.curGap2 >= triggerInfo.targetGap2) || (triggerInfo.secondaryToothCount == 5) )
      {
        triggerInfo.secondaryToothCount = 1;
        triggerInfo.revolutionOne = 1; //Sequential revolution reset
        triggerInfo.triggerSecFilterTime = 0; //This is used to prevent a condition where serious intermittent signals (Eg someone furiously plugging the sensor wire in and out) can leave the filter in an unrecoverable state
      }
      else
      {
        triggerInfo.triggerSecFilterTime = triggerInfo.curGap2 >> 2; //Set filter at 25% of the current speed. Filter can only be recalculated for the regular teeth, not the missing one.
        triggerInfo.secondaryToothCount++;
      }

    triggerInfo.toothLastSecToothTime = triggerInfo.curTime2;

    //Record the VVT Angle
    //We use the first tooth after the long gap as our reference, this remains in the same engine
    //cycle even when the VVT is at either end of its full swing.
    if( (configPage6.vvtEnabled > 0) && (triggerInfo.revolutionOne == 1) && (triggerInfo.secondaryToothCount == 1) )
    {
      int16_t curAngle;
      curAngle = getCrankAngle();
      while(curAngle > 360) { curAngle -= 360; }
      if( configPage6.vvtMode == VVT_MODE_CLOSED_LOOP )
      {
        curAngle = LOW_PASS_FILTER( (curAngle << 1), configPage4.ANGLEFILTER_VVT, curAngle);
        currentStatus.vvt1Angle = 360 - curAngle - configPage10.vvtCL0DutyAng;
      }
    }
  } //Trigger filter
}

uint16_t getRPM_FordST170(void)
{
  uint16_t tempRPM = 0;
  if( currentStatus.RPM < currentStatus.crankRPM )
  {
    if(triggerInfo.toothCurrentCount != 1)
    {
      tempRPM = crankingGetRPM(36, CRANK_SPEED);
    }
    else { tempRPM = currentStatus.RPM; } //Can't do per tooth RPM if we're at tooth #1 as the missing tooth messes the calculation
  }
  else
  {
    tempRPM = stdGetRPM(CRANK_SPEED);
  }
  return tempRPM;
}

int getCrankAngle_FordST170(void)
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

static uint16_t __attribute__((noinline)) calcSetEndTeeth_FordST170(int ignitionAngle, uint8_t toothAdder) {
  int16_t tempEndTooth = ignitionAngle - configPage4.triggerAngle;
#ifdef USE_LIBDIVIDE
  tempEndTooth = libdivide::libdivide_s16_do(tempEndTooth, &divtriggerInfo.triggerToothAngle);
#else
  tempEndTooth = tempEndTooth / (int16_t)triggerInfo.triggerToothAngle;
#endif
  tempEndTooth = nudge(1, 36U + toothAdder,  tempEndTooth - 1, 36U + toothAdder);
  return clampToActualTeeth((uint16_t)tempEndTooth, toothAdder);
}

void triggerSetEndTeeth_FordST170(void)
{
  byte toothAdder = 0;
   if( (configPage4.sparkMode == IGN_MODE_SEQUENTIAL) && (configPage4.TrigSpeed == CRANK_SPEED) ) { toothAdder = 36; }

  triggerInfo.ignition1EndTooth = calcSetEndTeeth_FordST170(ignition1EndAngle, toothAdder);
  triggerInfo.ignition2EndTooth = calcSetEndTeeth_FordST170(ignition2EndAngle, toothAdder);
  triggerInfo.ignition3EndTooth = calcSetEndTeeth_FordST170(ignition3EndAngle, toothAdder);
  triggerInfo.ignition4EndTooth = calcSetEndTeeth_FordST170(ignition4EndAngle, toothAdder);

  // Removed ign channels >4 as an ST170 engine is a 4 cylinder
}
/** @} */

