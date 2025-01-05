


void triggerSetup_DRZ400(void)
{
  triggerInfo.triggerToothAngle = 360 / configPage4.triggerTeeth; //The number of degrees that passes from tooth to tooth
  if(configPage4.TrigSpeed == 1) { triggerInfo.triggerToothAngle = 720 / configPage4.triggerTeeth; } //Account for cam speed
  triggerInfo.toothCurrentCount = UINT8_MAX; //Default value
  triggerInfo.triggerFilterTime = (MICROS_PER_SEC / (MAX_RPM / 60U * configPage4.triggerTeeth)); //Trigger filter time is the shortest possible time (in uS) that there can be between crank teeth (ie at max RPM). Any pulses that occur faster than this time will be discarded as noise
  triggerInfo.triggerSecFilterTime = (MICROS_PER_SEC / (MAX_RPM / 60U * 2U)); //Same as above, but fixed at 2 teeth on the secondary input
  BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_2ND_DERIV);
  BIT_SET(triggerInfo.decoderState, BIT_DECODER_IS_SEQUENTIAL);
  BIT_SET(triggerInfo.decoderState, BIT_DECODER_TOOTH_ANG_CORRECT); //This is always true for this pattern
  BIT_SET(triggerInfo.decoderState, BIT_DECODER_HAS_SECONDARY);
  MAX_STALL_TIME = ((MICROS_PER_DEG_1_RPM/50U) * triggerInfo.triggerToothAngle); //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
}

void triggerSec_DRZ400(void)
{
  triggerInfo.curTime2 = micros();
  triggerInfo.curGap2 = triggerInfo.curTime2 - triggerInfo.toothLastSecToothTime;
  if ( triggerInfo.curGap2 >= triggerInfo.triggerSecFilterTime )
  {
    triggerInfo.toothLastSecToothTime = triggerInfo.curTime2;

    if(currentStatus.hasSync == false)
    {
      triggerInfo.toothLastToothTime = micros();
      triggerInfo.toothLastMinusOneToothTime = micros() - ((MICROS_PER_MIN/10U) / configPage4.triggerTeeth); //Fixes RPM at 10rpm until a full revolution has taken place
      triggerInfo.toothCurrentCount = configPage4.triggerTeeth;
      currentStatus.syncLossCounter++;
      currentStatus.hasSync = true;
    }
    else
    {
      // have rotation, set tooth to six so next tooth is 1 & duel wheel rotation code kicks in
      triggerInfo.toothCurrentCount = 6;
    }
  }

  triggerInfo.triggerSecFilterTime = (triggerInfo.toothOneTime - triggerInfo.toothOneMinusOneTime) >> 1; //Set filter at 50% of the current crank speed.
}


