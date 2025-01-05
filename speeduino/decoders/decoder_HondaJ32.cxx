

/** @} */
/** Honda J 32 (3.2 liter 6 cyl SOHC).
*  The Honda J32a4 (and all J series I'm aware of) has a crank trigger with nominal 24 teeth (22 teeth actually present).
*  It has one missing tooth, then 7 teeth, then another missing tooth, then 15 teeth.
*  The tooth rising edges all have uniform spacing between them, except for teeth 14 and 22, which measure about
*  18 degrees between rising edges, rather than 15 degrees as the other teeth do.  These slightly larger
*  teeth are immediately before a gap, and the extra 3 degrees is made up for in the gap, the gap being about
*  3 degrees smaller than might be nominally expected, such that the expected rotational angle is restored immediately after
*  the gap (missing tooth) passes.
*  Teeth are represented as 0V at the ECU, no teeth are represented as 5V.
*  Top dead center of cylinder number 1 occurs as we lose sight of (just pass) the first tooth in the string of 15 teeth
*  (this is a rising edge).
*  The second tooth in the string of 15 teeth is designated as tooth 1 in this code. This means that
*  when the interrupt for tooth 1 fires (as we just pass tooth 1), crank angle = 360/24 = 15 degrees.
*  It follows that the first tooth in the string of 7 teeth is tooth 16.
* @defgroup dec_honda_j_32 Honda J 32
* @{
*/
void triggerSetup_HondaJ32(void)
{
  triggerInfo.triggerToothAngle = 360 / 24; //The number of degrees that passes from tooth to tooth
  MAX_STALL_TIME = ((MICROS_PER_DEG_1_RPM/10U) * triggerInfo.triggerToothAngle); //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
  BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_2ND_DERIV);
  BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_IS_SEQUENTIAL);
  BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_HAS_SECONDARY);

  // Filter (ignore) triggers that are faster than this.
  triggerInfo.triggerFilterTime = (MICROS_PER_SEC / (MAX_RPM / 60 * 24));
  triggerInfo.toothLastToothTime = 0;
  triggerInfo.toothCurrentCount = 0;
  triggerInfo.toothOneTime = 0;
  triggerInfo.toothOneMinusOneTime = 0;
  triggerInfo.lastGap = 0;
  triggerInfo.revolutionOne = 0;
}

void triggerPri_HondaJ32(void)
{
  // This function is called only on rising edges, which occur as we lose sight of a tooth.
  // This function sets the following state variables for use in other functions:
  // triggerInfo.toothLastToothTime, triggerInfo.toothOneTime, triggerInfo.revolutionOne (just toggles - not correct)
  triggerInfo.curTime = micros();
  triggerInfo.curGap = triggerInfo.curTime - triggerInfo.toothLastToothTime;
  triggerInfo.toothLastToothTime = triggerInfo.curTime;

  BIT_SET(triggerInfo.decoderState, BIT_DECODER_VALID_TRIGGER); //Flag this pulse as being a valid trigger (ie that it passed filters)

  if (currentStatus.hasSync == true) // We have sync
  {
    triggerInfo.toothCurrentCount++;

    if (triggerInfo.toothCurrentCount == 25) { // handle rollover.  Normal sized tooth here
      triggerInfo.toothCurrentCount = 1;
      triggerInfo.toothOneMinusOneTime = triggerInfo.toothOneTime;
      triggerInfo.toothOneTime = triggerInfo.curTime;
      currentStatus.startRevolutions++;
      SetRevolutionTime(triggerInfo.toothOneTime - triggerInfo.toothOneMinusOneTime);
    }
    else if (triggerInfo.toothCurrentCount == 23 || triggerInfo.toothCurrentCount == 15) // This is the first tooth after a missing tooth
    {
      triggerInfo.toothCurrentCount++; // account for missing tooth
      if (triggerInfo.curGap < (triggerInfo.lastGap >> 1) * 3) // This should be a big gap, if we find it's not actually big, we lost sync
      {
        currentStatus.hasSync = false;
        triggerInfo.toothCurrentCount=1;
      }
    }
    else if (triggerInfo.toothCurrentCount != 14 && triggerInfo.toothCurrentCount != 22)
    {
      // Teeth 14 and 22 are about 18 rather than 15 degrees so don't update last_gap with this unusual spacing
      triggerInfo.lastGap = triggerInfo.curGap;
    }
    // else triggerInfo.toothCurrentCount == 14 or 22.  Take no further action.
  }
  else // we do not have sync yet. While syncing, treat tooth 14 and 22 as normal teeth
  {
    if (triggerInfo.curGap < (triggerInfo.lastGap >> 1) * 3 || triggerInfo.lastGap == 0){ // Regular tooth, triggerInfo.lastGap == 0 at startup
      triggerInfo.toothCurrentCount++;  // Increment teeth between gaps
      triggerInfo.lastGap = triggerInfo.curGap;
    }
    else { // First tooth after the missing tooth
      if (triggerInfo.toothCurrentCount == 15) {  // 15 teeth since the gap before this, meaning we just passed the second gap and are synced
        currentStatus.hasSync = true;
        triggerInfo.toothCurrentCount = 16;  // This so happens to be the tooth number of the first tooth in the string of 7 (where we are now)
        triggerInfo.toothOneTime = triggerInfo.curTime - (15 * triggerInfo.lastGap); // Initialize tooth 1 times based on last gap width.
        triggerInfo.toothOneMinusOneTime = triggerInfo.toothOneTime - (24 * triggerInfo.lastGap);
      }
      else{ // Unclear which gap we just passed. reset counter
        triggerInfo.toothCurrentCount = 1;
      }
    }
  }
}

// There's currently no compelling reason to implement cam timing on the J32. (Have to do semi-sequential injection, wasted spark, there is no VTC on this engine, just VTEC)
void triggerSec_HondaJ32(void)
{
  return;
}

uint16_t getRPM_HondaJ32(void)
{
  return RpmFromRevolutionTimeUs(revolutionTime); // revolutionTime set by SetRevolutionTime()
}

int getCrankAngle_HondaJ32(void)
{
  // Returns values from 0 to 360.
  // Tooth 1 time occurs 360/24 degrees after TDC.
  // Teeth 14 and 22 are unusually sized (18 degrees), but the missing tooth is smaller (12 degrees), so this oddity only applies when triggerInfo.toothCurrentCount = 14 || 22
  int crankAngle;
  uint16_t temptoothCurrentCount;
  noInterrupts();
    temptoothCurrentCount = triggerInfo.toothCurrentCount;
    triggerInfo.lastCrankAngleCalc = micros(); //micros() is no longer interrupt safe
    triggerInfo.elapsedTime = triggerInfo.lastCrankAngleCalc - triggerInfo.toothLastToothTime;
  interrupts();

  if (temptoothCurrentCount == 14)
  {
    crankAngle = 213; // 13 teeth * 15 degrees/tooth + 18 degrees
  }
  else if (temptoothCurrentCount == 22)
  {
    crankAngle = 333; // 21 teeth * 15 degrees/tooth + 18 degrees
  }
  else
  {
    crankAngle = triggerInfo.triggerToothAngle * temptoothCurrentCount;
  }
  crankAngle += timeToAngleDegPerMicroSec(triggerInfo.elapsedTime) + configPage4.triggerAngle;

  if (crankAngle >= 720) { crankAngle -= 720; }
  if (crankAngle > CRANK_ANGLE_MAX) { crankAngle -= CRANK_ANGLE_MAX; }
  if (crankAngle < 0) { crankAngle += 360; }

  return crankAngle;
}

void triggerSetEndTeeth_HondaJ32(void)
{
  return;
}

/** @} */

