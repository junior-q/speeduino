/** Yamaha Vmax 1990+ with 6 uneven teeth, triggering on the wide lobe.
Within the decoder code, the sync tooth is referred to as tooth #1. Derived from Harley and made to work on the Yamah Vmax.
Trigger is based on 'CHANGE' so we get a signal on the up and downward edges of the lobe. This is required to identify the wide lobe.
* @defgroup dec_vmax Yamaha Vmax
* @{
*/
void triggerSetup_Vmax(void)
{
  triggerInfo.triggerToothAngle = 0; // The number of degrees that passes from tooth to tooth, ev. 0. It alternates uneven
  BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_2ND_DERIV);
  BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_IS_SEQUENTIAL);
  BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_HAS_SECONDARY);
  MAX_STALL_TIME = ((MICROS_PER_DEG_1_RPM/50U) * 60U); //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
  if(currentStatus.initialisationComplete == false) { triggerInfo.toothLastToothTime = micros(); } //Set a startup value here to avoid filter errors when starting. This MUST have the initi check to prevent the fuel pump just staying on all the time
  triggerInfo.triggerFilterTime = 1500;
  BIT_SET(triggerInfo.decoderState, BIT_DECODER_VALID_TRIGGER); // We must start with a valid trigger or we cannot start measuring the lobe width. We only have a false trigger on the lobe up event when it doesn't pass the filter. Then, the lobe width will also not be beasured.
  triggerInfo.toothAngles[1] = 0;      //tooth #1, these are the absolute tooth positions
  triggerInfo.toothAngles[2] = 40;     //tooth #2
  triggerInfo.toothAngles[3] = 110;    //tooth #3
  triggerInfo.toothAngles[4] = 180;    //tooth #4
  triggerInfo.toothAngles[5] = 220;    //tooth #5
  triggerInfo.toothAngles[6] = 290;    //tooth #6
}

//triggerInfo.curGap = microseconds between primary triggers
//triggerInfo.curGap2 = microseconds between secondary triggers
//triggerInfo.toothCurrentCount = the current number for the end of a lobe
//triggerInfo.secondaryToothCount = the current number of the beginning of a lobe
//We measure the width of a lobe so on the end of a lobe, but want to trigger on the beginning. Variable triggerInfo.toothCurrentCount tracks the downward events, and triggerInfo.secondaryToothCount updates on the upward events. Ideally, it should be the other way round but the engine stall routine resets triggerInfo.secondaryToothCount, so it would not sync again after an engine stall.

void triggerPri_Vmax(void)
{
  triggerInfo.curTime = micros();
  if(READ_PRI_TRIGGER() == primaryTriggerEdge){// Forwarded from the config page to setup the primary trigger edge (rising or falling). Inverting VR-conditioners require FALLING, non-inverting VR-conditioners require RISING in the Trigger edge setup.
    triggerInfo.curGap2 = triggerInfo.curTime;
    triggerInfo.curGap = triggerInfo.curTime - triggerInfo.toothLastToothTime;
    if ( (triggerInfo.curGap >= triggerInfo.triggerFilterTime) ){
      BIT_SET(triggerInfo.decoderState, BIT_DECODER_VALID_TRIGGER); //Flag this pulse as being a valid trigger (ie that it passed filters)
      if (triggerInfo.toothCurrentCount > 0) // We have sync based on the tooth width.
      {
          BIT_SET(triggerInfo.decoderState, BIT_DECODER_VALID_TRIGGER); //Flag this pulse as being a valid trigger (ie that it passed filters)
          if (triggerInfo.toothCurrentCount==1)
          {
            triggerInfo.secondaryToothCount = 1;
            triggerInfo.triggerToothAngle = 70;// Has to be equal to Angle Routine, and describe the delta between two teeth.
            triggerInfo.toothOneMinusOneTime = triggerInfo.toothOneTime;
            triggerInfo.toothOneTime = triggerInfo.curTime;
            currentStatus.hasSync = true;
            //setFilter((triggerInfo.curGap/1.75));//Angle to this tooth is 70, next is in 40, compensating.
            setFilter( ((triggerInfo.curGap*4)/7) );//Angle to this tooth is 70, next is in 40, compensating.
            currentStatus.startRevolutions++; //Counter
          }
          else if (triggerInfo.toothCurrentCount==2)
          {
            triggerInfo.secondaryToothCount = 2;
            triggerInfo.triggerToothAngle = 40;
            //setFilter((triggerInfo.curGap*1.75));//Angle to this tooth is 40, next is in 70, compensating.
            setFilter( ((triggerInfo.curGap*7)/4) );//Angle to this tooth is 40, next is in 70, compensating.
          }
          else if (triggerInfo.toothCurrentCount==3)
          {
            triggerInfo.secondaryToothCount = 3;
            triggerInfo.triggerToothAngle = 70;
            setFilter(triggerInfo.curGap);//Angle to this tooth is 70, next is in 70. No need to compensate.
          }
          else if (triggerInfo.toothCurrentCount==4)
          {
            triggerInfo.secondaryToothCount = 4;
            triggerInfo.triggerToothAngle = 70;
            //setFilter((triggerInfo.curGap/1.75));//Angle to this tooth is 70, next is in 40, compensating.
            setFilter( ((triggerInfo.curGap*4)/7) );//Angle to this tooth is 70, next is in 40, compensating.
          }
          else if (triggerInfo.toothCurrentCount==5)
          {
            triggerInfo.secondaryToothCount = 5;
            triggerInfo.triggerToothAngle = 40;
            //setFilter((triggerInfo.curGap*1.75));//Angle to this tooth is 40, next is in 70, compensating.
            setFilter( ((triggerInfo.curGap*7)/4) );//Angle to this tooth is 40, next is in 70, compensating.
          }
          else if (triggerInfo.toothCurrentCount==6)
          {
            triggerInfo.secondaryToothCount = 6;
            triggerInfo.triggerToothAngle = 70;
            setFilter(triggerInfo.curGap);//Angle to this tooth is 70, next is in 70. No need to compensate.
          }
          triggerInfo.toothLastMinusOneToothTime = triggerInfo.toothLastToothTime;
          triggerInfo.toothLastToothTime = triggerInfo.curTime;
          if (triggerInfo.triggerFilterTime > 50000){//The first pulse seen
            triggerInfo.triggerFilterTime = 0;
          }
      }
      else{
        triggerInfo.triggerFilterTime = 0;
        return;//Zero, no sync yet.
      }
    }
    else{
      BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_VALID_TRIGGER); //Flag this pulse as being an invalid trigger
    }
  }
  else if( BIT_CHECK(triggerInfo.decoderState, BIT_DECODER_VALID_TRIGGER) ) // Inverted due to vr conditioner. So this is the falling lobe. We only process if there was a valid trigger.
  {
    uint32_t curGapLocal = triggerInfo.curTime - triggerInfo.curGap2;

    if (curGapLocal > (triggerInfo.lastGap * 2)){// Small lobe is 5 degrees, big lobe is 45 degrees. So this should be the wide lobe.
        if (triggerInfo.toothCurrentCount == 0 || triggerInfo.toothCurrentCount == 6){//Wide should be seen with triggerInfo.toothCurrentCount = 0, when there is no sync yet, or triggerInfo.toothCurrentCount = 6 when we have done a full revolution.
          currentStatus.hasSync = true;
        }
        else{//Wide lobe seen where it shouldn't, adding a sync error.
          currentStatus.syncLossCounter++;
        }
        triggerInfo.toothCurrentCount = 1;
    }
    else if(triggerInfo.toothCurrentCount == 6){//The 6th lobe should be wide, adding a sync error.
        triggerInfo.toothCurrentCount = 1;
        currentStatus.syncLossCounter++;
    }
    else{// Small lobe, just add 1 to the triggerInfo.toothCurrentCount.
      triggerInfo.toothCurrentCount++;
    }
    triggerInfo.lastGap = curGapLocal;
    return;
  }
  else if( BIT_CHECK(triggerInfo.decoderState, BIT_DECODER_VALID_TRIGGER) == false)
  {
    BIT_SET(triggerInfo.decoderState, BIT_DECODER_VALID_TRIGGER); //We reset this every time to ensure we only filter when needed.
  }
}


void triggerSec_Vmax(void)
// Needs to be enabled in main()
{
  return;// No need for now. The only thing it could help to sync more quickly or confirm position.
} // End Sec Trigger


uint16_t getRPM_Vmax(void)
{
  uint16_t tempRPM = 0;
  if (currentStatus.hasSync == true)
  {
    if ( currentStatus.RPM < (unsigned int)(configPage4.crankRPM * 100) )
    {
      int tempToothAngle;
      unsigned long toothTime;
      if ( (triggerInfo.toothLastToothTime == 0) || (triggerInfo.toothLastMinusOneToothTime == 0) ) { tempRPM = 0; }
      else
      {
        noInterrupts();
        tempToothAngle = triggerInfo.triggerToothAngle;
        SetRevolutionTime(triggerInfo.toothOneTime - triggerInfo.toothOneMinusOneTime); //The time in uS that one revolution would take at current speed (The time tooth 1 was last seen, minus the time it was seen prior to that)
        toothTime = (triggerInfo.toothLastToothTime - triggerInfo.toothLastMinusOneToothTime);
        interrupts();
        toothTime = toothTime * 36;
        tempRPM = ((unsigned long)tempToothAngle * (MICROS_PER_MIN/10U)) / toothTime;
      }
    }
    else {
      tempRPM = stdGetRPM(CRANK_SPEED);
    }
  }
  return tempRPM;
}


int getCrankAngle_Vmax(void)
{
  //This is the current angle ATDC the engine is at. This is the last known position based on what tooth was last 'seen'. It is only accurate to the resolution of the trigger wheel (Eg 36-1 is 10 degrees)
  unsigned long temptoothLastToothTime;
  int tempsecondaryToothCount;
  //Grab some variables that are used in the trigger code and assign them to temp variables.
  noInterrupts();
  tempsecondaryToothCount = triggerInfo.secondaryToothCount;
  temptoothLastToothTime = triggerInfo.toothLastToothTime;
  triggerInfo.lastCrankAngleCalc = micros(); //micros() is no longer interrupt safe
  interrupts();

  //Check if the last tooth seen was the reference tooth (Number 3). All others can be calculated, but tooth 3 has a unique angle
  int crankAngle;
  crankAngle=triggerInfo.toothAngles[tempsecondaryToothCount] + configPage4.triggerAngle;

  //Estimate the number of degrees travelled since the last tooth}
  triggerInfo.elapsedTime = (triggerInfo.lastCrankAngleCalc - temptoothLastToothTime);
  crankAngle += timeToAngleDegPerMicroSec(triggerInfo.elapsedTime);

  if (crankAngle >= 720) { crankAngle -= 720; }
  if (crankAngle < 0) { crankAngle += 360; }

  return crankAngle;
}

void triggerSetEndTeeth_Vmax(void)
{
}

/** @} */


