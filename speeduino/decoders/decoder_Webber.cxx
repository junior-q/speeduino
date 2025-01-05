
/** Weber-Marelli trigger setup with 2 wheels, 4 teeth 90deg apart on crank and 2 90deg apart on cam.
Uses DualWheel decoders, There can be no missing teeth on the primary wheel.
* @defgroup dec_weber_marelli Weber-Marelli
* @{
*/
void triggerPri_Webber(void)
{
  triggerInfo.curTime = micros();
  triggerInfo.curGap = triggerInfo.curTime - triggerInfo.toothLastToothTime;
  if ( triggerInfo.curGap >= triggerInfo.triggerFilterTime )
  {
    triggerInfo.toothCurrentCount++; //Increment the tooth counter
    if (triggerInfo.checkSyncToothCount > 0) { triggerInfo.checkSyncToothCount++; }
    if ( triggerInfo.triggerSecFilterTime <= triggerInfo.curGap ) { triggerInfo.triggerSecFilterTime = triggerInfo.curGap + (triggerInfo.curGap>>1); } //150% crank tooth
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
      }

      setFilter(triggerInfo.curGap); //Recalc the new filter value
    }
    else
    {
      if ( (triggerInfo.secondaryToothCount == 1) && (triggerInfo.checkSyncToothCount == 4) )
      {
        triggerInfo.toothCurrentCount = 2;
        currentStatus.hasSync = true;
        triggerInfo.revolutionOne = 0; //Sequential revolution reset
      }
    }

    //NEW IGNITION MODE
    if( (configPage2.perToothIgn == true) && (!BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK)) )
    {
      int16_t crankAngle = ( (triggerInfo.toothCurrentCount-1) * triggerInfo.triggerToothAngle ) + configPage4.triggerAngle;
      if( (configPage4.sparkMode == IGN_MODE_SEQUENTIAL) && (triggerInfo.revolutionOne == true) && (configPage4.TrigSpeed == CRANK_SPEED) )
      {
        crankAngle += 360;
        checkPerToothTiming(crankAngle, (configPage4.triggerTeeth + triggerInfo.toothCurrentCount));
      }
      else{ checkPerToothTiming(crankAngle, triggerInfo.toothCurrentCount); }
    }
  } //Trigger filter
}

void triggerSec_Webber(void)
{
  triggerInfo.curTime2 = micros();
  triggerInfo.curGap2 = triggerInfo.curTime2 - triggerInfo.toothLastSecToothTime;

  if ( triggerInfo.curGap2 >= triggerInfo.triggerSecFilterTime )
  {
    triggerInfo.toothLastSecToothTime = triggerInfo.curTime2;

    if ( (triggerInfo.secondaryToothCount == 2) && (triggerInfo.checkSyncToothCount == 3) )
    {
      if(currentStatus.hasSync == false)
      {
        triggerInfo.toothLastToothTime = micros();
        triggerInfo.toothLastMinusOneToothTime = micros() - 1500000; //Fixes RPM at 10rpm until a full revolution has taken place
        triggerInfo.toothCurrentCount = configPage4.triggerTeeth-1;

        currentStatus.hasSync = true;
      }
      else
      {
        if ( (triggerInfo.toothCurrentCount != (configPage4.triggerTeeth-1U)) && (currentStatus.startRevolutions > 2U)) { currentStatus.syncLossCounter++; } //Indicates likely sync loss.
        if (configPage4.useResync == 1) { triggerInfo.toothCurrentCount = configPage4.triggerTeeth-1; }
      }
      triggerInfo.revolutionOne = 1; //Sequential revolution reset
      triggerInfo.triggerSecFilterTime = triggerInfo.curGap << 2; //4 crank teeth
      triggerInfo.secondaryToothCount = 1; //Next tooth should be first
    } //Running, on first CAM pulse restart crank teeth count, on second the counter should be 3
    else if ( (currentStatus.hasSync == false) && (triggerInfo.toothCurrentCount >= 3) && (triggerInfo.secondaryToothCount == 0) )
    {
      triggerInfo.toothLastToothTime = micros();
      triggerInfo.toothLastMinusOneToothTime = micros() - 1500000; //Fixes RPM at 10rpm until a full revolution has taken place
      triggerInfo.toothCurrentCount = 1;
      triggerInfo.revolutionOne = 1; //Sequential revolution reset

      currentStatus.hasSync = true;
    } //First start, between gaps on CAM pulses have 2 teeth, sync on first CAM pulse if seen 3 teeth or more
    else
    {
      triggerInfo.triggerSecFilterTime = triggerInfo.curGap + (triggerInfo.curGap>>1); //150% crank tooth
      triggerInfo.secondaryToothCount++;
      triggerInfo.checkSyncToothCount = 1; //Tooth 1 considered as already been seen
    } //First time might fall here, second CAM tooth will
  }
  else
  {
    triggerInfo.triggerSecFilterTime = triggerInfo.curGap + (triggerInfo.curGap>>1); //Noise region, using 150% of crank tooth
    triggerInfo.checkSyncToothCount = 1; //Reset tooth counter
  } //Trigger filter
}
/** @} */

