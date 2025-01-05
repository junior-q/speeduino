
/** Nissan 360 tooth on cam (Optical trigger disc inside distributor housing).
See http://wiki.r31skylineclub.com/index.php/Crank_Angle_Sensor .
* @defgroup dec_nissan360 Nissan 360 tooth on cam
* @{
*/
void triggerSetup_Nissan360(void)
{
  triggerInfo.triggerFilterTime = (MICROS_PER_SEC / (MAX_RPM / 60U * 360UL)); //Trigger filter time is the shortest possible time (in uS) that there can be between crank teeth (ie at max RPM). Any pulses that occur faster than this time will be discarded as noise
  triggerInfo.triggerSecFilterTime = (int)(MICROS_PER_SEC / (MAX_RPM / 60U * 2U)) / 2U; //Same as above, but fixed at 2 teeth on the secondary input and divided by 2 (for cam speed)
  triggerInfo.secondaryToothCount = 0; //Initially set to 0 prior to calculating the secondary window duration
  BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_2ND_DERIV);
  BIT_SET(triggerInfo.decoderState, BIT_DECODER_IS_SEQUENTIAL);
  BIT_SET(triggerInfo.decoderState, BIT_DECODER_HAS_SECONDARY);
  triggerInfo.toothCurrentCount = 1;
  triggerInfo.triggerToothAngle = 2;
  MAX_STALL_TIME = ((MICROS_PER_DEG_1_RPM/50U) * triggerInfo.triggerToothAngle); //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
}


void triggerPri_Nissan360(void)
{
   triggerInfo.curTime = micros();
   triggerInfo.curGap = triggerInfo.curTime - triggerInfo.toothLastToothTime;
   //if ( triggerInfo.curGap < triggerInfo.triggerFilterTime ) { return; }
   triggerInfo.toothCurrentCount++; //Increment the tooth counter
   BIT_SET(triggerInfo.decoderState, BIT_DECODER_VALID_TRIGGER); //Flag this pulse as being a valid trigger (ie that it passed filters)

   triggerInfo.toothLastMinusOneToothTime = triggerInfo.toothLastToothTime;
   triggerInfo.toothLastToothTime = triggerInfo.curTime;

   if ( currentStatus.hasSync == true )
   {
     if ( triggerInfo.toothCurrentCount == 361 ) //2 complete crank revolutions
     {
       triggerInfo.toothCurrentCount = 1;
       triggerInfo.toothOneMinusOneTime = triggerInfo.toothOneTime;
       triggerInfo.toothOneTime = triggerInfo.curTime;
       currentStatus.startRevolutions++; //Counter
     }
     //Recalc the new filter value
     //setFilter(triggerInfo.curGap);

     //EXPERIMENTAL!
     if(configPage2.perToothIgn == true)
     {
        int16_t crankAngle = ( (triggerInfo.toothCurrentCount-1) * 2 ) + configPage4.triggerAngle;
        if(crankAngle > CRANK_ANGLE_MAX_IGN)
        {
          crankAngle -= CRANK_ANGLE_MAX_IGN;
          checkPerToothTiming(crankAngle, (triggerInfo.toothCurrentCount/2) );
        }
        else
        {
          checkPerToothTiming(crankAngle, triggerInfo.toothCurrentCount);
        }

     }
   }
}

void triggerSec_Nissan360(void)
{
  triggerInfo.curTime2 = micros();
  triggerInfo.curGap2 = triggerInfo.curTime2 - triggerInfo.toothLastSecToothTime;
  //if ( triggerInfo.curGap2 < triggerInfo.triggerSecFilterTime ) { return; }
  triggerInfo.toothLastSecToothTime = triggerInfo.curTime2;
  //OPTIONAL: Set filter at 25% of the current speed
  //triggerInfo.triggerSecFilterTime = triggerInfo.curGap2 >> 2;


  //Calculate number of primary teeth that this window has been active for
  byte trigEdge;
  if(configPage4.TrigEdgeSec == 0) { trigEdge = LOW; }
  else { trigEdge = HIGH; }

  if( (triggerInfo.secondaryToothCount == 0) || (READ_SEC_TRIGGER() == trigEdge) ) { triggerInfo.secondaryToothCount = triggerInfo.toothCurrentCount; } //This occurs on the first rotation upon powerup OR the start of a secondary window
  else
  {
    //If we reach here, we are at the end of a secondary window
    byte secondaryDuration = triggerInfo.toothCurrentCount - triggerInfo.secondaryToothCount; //How many primary teeth have passed during the duration of this secondary window

    if(currentStatus.hasSync == false)
    {
      if(configPage2.nCylinders == 4)
      {
        //Supported pattern is where all the inner windows as a different size (Most SR engines)
        //These equate to 4,8,12,16 teeth spacings
        if( (secondaryDuration >= 15) && (secondaryDuration <= 17) ) //Duration of window = 16 primary teeth
        {
          triggerInfo.toothCurrentCount = 16; //End of first window (The longest) occurs 16 teeth after TDC
          currentStatus.hasSync = true;
        }
        else if( (secondaryDuration >= 11) && (secondaryDuration <= 13) ) //Duration of window = 12 primary teeth
        {
          triggerInfo.toothCurrentCount = 102; //End of second window is after 90+12 primary teeth
          currentStatus.hasSync = true;
        }
        else if( (secondaryDuration >= 7) && (secondaryDuration <= 9) ) //Duration of window = 8 primary teeth
        {
          triggerInfo.toothCurrentCount = 188; //End of third window is after 90+90+8 primary teeth
          currentStatus.hasSync = true;
        }
        else if( (secondaryDuration >= 3) && (secondaryDuration <= 5) ) //Duration of window = 4 primary teeth
        {
          triggerInfo.toothCurrentCount = 274; //End of fourth window is after 90+90+90+4 primary teeth
          currentStatus.hasSync = true;
        }
        else { currentStatus.hasSync = false; currentStatus.syncLossCounter++; } //This should really never happen
      }
      else if(configPage2.nCylinders == 6)
      {
        //Pattern on the 6 cylinders is 4-8-12-16-20-24
        if( (secondaryDuration >= 3) && (secondaryDuration <= 5) ) //Duration of window = 4 primary teeth
        {
          triggerInfo.toothCurrentCount = 124; //End of smallest window is after 60+60+4 primary teeth
          currentStatus.hasSync = true;
        }
      }
      else if(configPage2.nCylinders == 8)
      {
        //V8 Optispark
        //Pattern on the 8 cylinders is the same as the 6 cylinder 4-8-12-16-20-24
        if( (secondaryDuration >= 6) && (secondaryDuration <= 8) ) //Duration of window = 16 primary teeth
        {
          triggerInfo.toothCurrentCount = 56; //End of the shortest of the individual windows. Occurs at 102 crank degrees.
          currentStatus.hasSync = true;
        }
      }
      else { currentStatus.hasSync = false; } //This should really never happen (Only 4, 6 and 8 cylinder engines for this pattern)
    }
    else
    {
      if (configPage4.useResync == true)
      {
        //Already have sync, but do a verify every 720 degrees.
        if(configPage2.nCylinders == 4)
        {
          if( (secondaryDuration >= 15) && (secondaryDuration <= 17) ) //Duration of window = 16 primary teeth
          {
            triggerInfo.toothCurrentCount = 16; //End of first window (The longest) occurs 16 teeth after TDC
          }
        }
        else if(configPage2.nCylinders == 6)
        {
          if(secondaryDuration == 4)
          {
            //triggerInfo.toothCurrentCount = 304;
          }
        } //Cylinder count
      } //use resync
    } //Has sync
  } //First getting sync or not
}

uint16_t getRPM_Nissan360(void)
{
  //Can't use stdGetRPM as there is no separate cranking RPM calc (stdGetRPM returns 0 if cranking)
  uint16_t tempRPM;
  if( (currentStatus.hasSync == true) && (triggerInfo.toothLastToothTime != 0) && (triggerInfo.toothLastMinusOneToothTime != 0) )
  {
    if(currentStatus.startRevolutions < 2)
    {
      noInterrupts();
      SetRevolutionTime((triggerInfo.toothLastToothTime - triggerInfo.toothLastMinusOneToothTime) * 180); //Each tooth covers 2 crank degrees, so multiply by 180 to get a full revolution time.
      interrupts();
    }
    else
    {
      noInterrupts();
      SetRevolutionTime((triggerInfo.toothOneTime - triggerInfo.toothOneMinusOneTime) >> 1); //The time in uS that one revolution would take at current speed (The time tooth 1 was last seen, minus the time it was seen prior to that)
      interrupts();
    }
    tempRPM = RpmFromRevolutionTimeUs(revolutionTime); //Calc RPM based on last full revolution time (Faster as /)
    MAX_STALL_TIME = revolutionTime << 1; //Set the stall time to be twice the current RPM. This is a safe figure as there should be no single revolution where this changes more than this
  }
  else { tempRPM = 0; }

  return tempRPM;
}


int getCrankAngle_Nissan360(void)
{
  //As each tooth represents 2 crank degrees, we only need to determine whether we're more or less than halfway between teeth to know whether to add another 1 degrees
  int crankAngle = 0;
  int temptoothLastToothTime;
  int temptoothLastMinusOneToothTime;
  int temptoothCurrentCount;

  noInterrupts();
  temptoothLastToothTime = triggerInfo.toothLastToothTime;
  temptoothLastMinusOneToothTime = triggerInfo.toothLastMinusOneToothTime;
  temptoothCurrentCount = triggerInfo.toothCurrentCount;
  triggerInfo.lastCrankAngleCalc = micros(); //micros() is no longer interrupt safe
  interrupts();

  crankAngle = ( (temptoothCurrentCount - 1) * 2) + configPage4.triggerAngle;
  unsigned long halfTooth = (temptoothLastToothTime - temptoothLastMinusOneToothTime) / 2;
  triggerInfo.elapsedTime = (triggerInfo.lastCrankAngleCalc - temptoothLastToothTime);
  if (triggerInfo.elapsedTime > halfTooth)
  {
    //Means we're over halfway to the next tooth, so add on 1 degree
    crankAngle += 1;
  }

  if (crankAngle >= 720) { crankAngle -= 720; }
  if (crankAngle < 0) { crankAngle += 360; }

  return crankAngle;
}

void triggerSetEndTeeth_Nissan360(void)
{
  //This uses 4 prior teeth, just to ensure there is sufficient time to set the schedule etc
  byte offset_teeth = 4;
  if((ignition1EndAngle - offset_teeth) > configPage4.triggerAngle) { triggerInfo.ignition1EndTooth = ( (ignition1EndAngle - configPage4.triggerAngle) / 2 ) - offset_teeth; }
  else { triggerInfo.ignition1EndTooth = ( (ignition1EndAngle + 720 - configPage4.triggerAngle) / 2 ) - offset_teeth; }
  if((ignition2EndAngle - offset_teeth) > configPage4.triggerAngle) { triggerInfo.ignition2EndTooth = ( (ignition2EndAngle - configPage4.triggerAngle) / 2 ) - offset_teeth; }
  else { triggerInfo.ignition2EndTooth = ( (ignition2EndAngle + 720 - configPage4.triggerAngle) / 2 ) - offset_teeth; }
  if((ignition3EndAngle - offset_teeth) > configPage4.triggerAngle) { triggerInfo.ignition3EndTooth = ( (ignition3EndAngle - configPage4.triggerAngle) / 2 ) - offset_teeth; }
  else { triggerInfo.ignition3EndTooth = ( (ignition3EndAngle + 720 - configPage4.triggerAngle) / 2 ) - offset_teeth; }
  if((ignition4EndAngle - offset_teeth) > configPage4.triggerAngle) { triggerInfo.ignition4EndTooth = ( (ignition4EndAngle - configPage4.triggerAngle) / 2 ) - offset_teeth; }
  else { triggerInfo.ignition4EndTooth = ( (ignition4EndAngle + 720 - configPage4.triggerAngle) / 2 ) - offset_teeth; }
}
/** @} */

