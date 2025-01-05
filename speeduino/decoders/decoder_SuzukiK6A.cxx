
/** Suzuki K6A 3 cylinder engine

* (See: https://www.msextra.com/forums/viewtopic.php?t=74614)
* @defgroup Suzuki_K6A Suzuki K6A
* @{
*/
void triggerSetup_SuzukiK6A(void)
{
  triggerInfo.triggerToothAngle = 90; //The number of degrees that passes from tooth to tooth (primary) - set to a value, needs to be set per tooth
  triggerInfo.toothCurrentCount = 99; //Fake tooth count represents no sync

  configPage4.TrigSpeed = CAM_SPEED;
  triggerInfo.triggerActualTeeth = 7;
  triggerInfo.toothCurrentCount = 1;
  triggerInfo.curGap = triggerInfo.curGap2 = triggerInfo.curGap3 = 0;

  if(currentStatus.initialisationComplete == false) { triggerInfo.toothLastToothTime = micros(); } //Set a startup value here to avoid filter errors when starting. This MUST have the initial check to prevent the fuel pump just staying on all the time
  else { triggerInfo.toothLastToothTime = 0; }
  triggerInfo.toothLastMinusOneToothTime = 0;

  // based on data in msextra page linked to above we can deduce,
  // gap between rising and falling edge of a normal 70 degree tooth is 48 degrees, this means the gap is 70 degrees - 48 degrees = 22 degrees.
  // assume this is constant for all similar sized gaps and teeth
  // sync tooth is 35 degrees - eyeball looks like the tooth is 50% tooth and 50% gap so guess its 17 degrees and 18 degrees.

  // coded every tooth here in case you want to try "change" setting on the trigger setup (this is defined in init.ino and what i've set it to, otherwise you need code to select rising or falling in init.ino (steal it from another trigger)).
  // If you don't want change then drop the 'falling' edges listed below and half the number of edges + reduce the triggerInfo.triggerActualTeeth
  // nb as you can edit the trigger offset using rising or falling edge setup below is irrelevant as you can adjust via the trigger offset to cover the difference.

  // not using triggerInfo.toothAngles[0] as i'm hoping it makes logic easier

  triggerInfo.toothAngles[0] = -70;                 // Wrap around to 650,
  triggerInfo.toothAngles[1] = 0;                   // 0 TDC cylinder 1,
  triggerInfo.toothAngles[2] = 170;                 // 170 - end of cylinder 1, start of cylinder 3, trigger ignition for cylinder 3 on this tooth
  triggerInfo.toothAngles[3] = 240;                 // 70 TDC cylinder 3
  triggerInfo.toothAngles[4] = 410;                 // 170  - end of cylinder 3, start of cylinder2, trigger ignition for cylinder 2 on this tooth
  triggerInfo.toothAngles[5] = 480;                 // 70 TDC cylinder 2
  triggerInfo.toothAngles[6] = 515;                 // 35 Additional sync tooth
  triggerInfo.toothAngles[7] = 650;                 // 135 end of cylinder 2, start of cylinder 1, trigger ignition for cylinder 1 on this tooth
  triggerInfo.toothAngles[8] = 720;                 // 70 - gap to rotation to TDC1. array item 1 and 8 are the same, code never gets here its for reference only


  MAX_STALL_TIME = (3333UL * triggerInfo.triggerToothAngle); //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
  triggerInfo.triggerFilterTime = 1500; //10000 rpm, assuming we're triggering on both edges off the crank tooth.
  triggerInfo.triggerSecFilterTime = 0; //Need to figure out something better for this
  BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_HAS_FIXED_CRANKING);
  BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);
  BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_HAS_SECONDARY); // never sure if we need to set this in this type of trigger
  BIT_CLEAR(currentStatus.status3, BIT_STATUS3_HALFSYNC); // we can never have half sync - its either full or none.
  BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_2ND_DERIV);
  BIT_SET(triggerInfo.decoderState, BIT_DECODER_IS_SEQUENTIAL);
}

void triggerPri_SuzukiK6A(void)
{
  triggerInfo.curTime = micros();
  triggerInfo.curGap = triggerInfo.curTime - triggerInfo.toothLastToothTime;
  if ( (triggerInfo.curGap >= triggerInfo.triggerFilterTime) || (currentStatus.startRevolutions == 0U) )
  {
    triggerInfo.toothCurrentCount++;
    BIT_SET(triggerInfo.decoderState, BIT_DECODER_VALID_TRIGGER); //Flag this pulse as being a valid trigger (ie that it passed filters)

    triggerInfo.toothLastMinusOneToothTime = triggerInfo.toothLastToothTime;
    triggerInfo.toothLastToothTime = triggerInfo.curTime;


    // now to figure out if its a normal tooth or the extra sync tooth
    // pattern is normally small tooth, big tooth, small tooth, big tooth. The extra tooth breaks the pattern go it goes, big tooth (triggerInfo.curGap3), small tooth(triggerInfo.curGap2), small tooth(triggerInfo.curGap)
    // reuse triggerInfo.curGap2 and triggerInfo.curGap3 (from secondary and Tertiary decoders) to store previous tooth sizes as not needed in this decoder.

    if ( (  triggerInfo.curGap <= triggerInfo.curGap2 ) &&
         ( triggerInfo.curGap2 <= triggerInfo.curGap3 ) )
    {
      // cur Gap is smaller than last gap & last gap is smaller than gap before that - means we must be on sync tooth
      triggerInfo.toothCurrentCount = 6; // set tooth counter to correct tooth
      currentStatus.hasSync = true;
    }

    triggerInfo.curGap3 = triggerInfo.curGap2; // update values for next time we're in the loop
    triggerInfo.curGap2 = triggerInfo.curGap;


    if( (triggerInfo.toothCurrentCount == (triggerInfo.triggerActualTeeth + 1U)) && currentStatus.hasSync == true  )
    {
      // seen enough teeth to have a revolution of the crank
      triggerInfo.toothCurrentCount = 1; //Reset the counter
      triggerInfo.toothOneMinusOneTime = triggerInfo.toothOneTime;
      triggerInfo.toothOneTime = triggerInfo.curTime;
      currentStatus.startRevolutions = currentStatus.startRevolutions + 2U; // increment for 2 revs as we do 720 degrees on the the crank
    }
    else if (triggerInfo.toothCurrentCount > (triggerInfo.triggerActualTeeth + 1U))
    {
      // Lost sync
      currentStatus.hasSync = false;
      currentStatus.syncLossCounter++;
      triggerInfo.triggerFilterTime = 0;
      triggerInfo.toothCurrentCount=0;
    } else {
      // We have sync, but are part way through a revolution
      // Nothing to do but keep MISRA happy.
    }

    // check gaps match with tooth to check we have sync
    // so if we *think* we've seen tooth 3 whose gap should be smaller than the previous tooth & it isn't,
    // then we've lost sync
    switch (triggerInfo.toothCurrentCount)
    {
      case 1:
      case 3:
      case 5:
      case 6:
        // current tooth gap is bigger than previous tooth gap = syncloss
        // eg tooth 3 should be smaller than tooth 2 gap, if its not then we've lost sync and the tooth 3 we've just seen isn't really tooth 3
        if (triggerInfo.curGap > triggerInfo.curGap2)
        {
          currentStatus.hasSync = false;
          currentStatus.syncLossCounter++;
          triggerInfo.triggerFilterTime = 0;
          triggerInfo.toothCurrentCount=2;
        }
        break;

      case 2:
      case 4:
      case 7:
      default:
        // current tooth gap is smaller than the previous tooth gap = syncloss
        // eg tooth 2 should be bigger than tooth 1, if its not then we've got syncloss
        if (triggerInfo.curGap < triggerInfo.curGap2)
        {
          currentStatus.hasSync = false;
          currentStatus.syncLossCounter++;
          triggerInfo.triggerFilterTime = 0;
          triggerInfo.toothCurrentCount=1;
        }
        break;
    }

    // Setup data to allow other areas of the system to work due to odd sized teeth - this could be merged with sync checking above, left separate to keep code clearer as its doing only one function at once
    // % of filter are not based on previous tooth size but expected next tooth size
    // triggerInfo.triggerToothAngle is the size of the previous tooth not the future tooth
    if (currentStatus.hasSync == true )
    {
      switch (triggerInfo.toothCurrentCount) // Set tooth angle based on previous gap and triggerInfo.triggerFilterTime based on previous gap and next gap
      {
        case 2:
        case 4:
          // equivalent of tooth 1 except we've not done rotation code yet so its 8
          // 170 degree tooth, next tooth is 70
          switch (configPage4.triggerFilter)
          {
            case 1: // 25 % 17 degrees
              triggerInfo.triggerFilterTime = rshift<3>(triggerInfo.curGap);
              break;
            case 2: // 50 % 35 degrees
              triggerInfo.triggerFilterTime = rshift<3>(triggerInfo.curGap) + rshift<4>(triggerInfo.curGap);
              break;
            case 3: // 75 % 52 degrees
              triggerInfo.triggerFilterTime = rshift<2>(triggerInfo.curGap) + rshift<4>(triggerInfo.curGap);
              break;
            default:
              triggerInfo.triggerFilterTime = 0;
              break;
          }
          break;

        case 5:
          // 70 degrees, next tooth is 35
          switch (configPage4.triggerFilter)
          {
            case 1: // 25 % 8 degrees
              triggerInfo.triggerFilterTime = rshift<3>(triggerInfo.curGap);
              break;
            case 2: // 50 % 17 degrees
              triggerInfo.triggerFilterTime = rshift<2>(triggerInfo.curGap);
              break;
            case 3: // 75 % 25 degrees
              triggerInfo.triggerFilterTime = rshift<2>(triggerInfo.curGap) + rshift<3>(triggerInfo.curGap);
              break;
            default:
              triggerInfo.triggerFilterTime = 0;
              break;
          }
          break;

        case 6:
          // sync tooth, next tooth is 135
          switch (configPage4.triggerFilter)
          {
            case 1: // 25 % 33 degrees
              triggerInfo.triggerFilterTime = triggerInfo.curGap;
              break;
            case 2: // 50 % 67 degrees
              triggerInfo.triggerFilterTime = triggerInfo.curGap * 2U;
              break;
            case 3: // 75 % 100 degrees
              triggerInfo.triggerFilterTime = triggerInfo.curGap * 3U;
              break;
            default:
              triggerInfo.triggerFilterTime = 0;
              break;
          }
          break;

        case 7:
          // 135 degre tooth, next tooth is 70
          switch (configPage4.triggerFilter)
          {
            case 1: // 25 % 17 degrees
              triggerInfo.triggerFilterTime = rshift<3>(triggerInfo.curGap);
              break;
            case 2: // 50 % 35 degrees
              triggerInfo.triggerFilterTime = rshift<2>(triggerInfo.curGap);
              break;
            case 3: // 75 % 52 degrees
              triggerInfo.triggerFilterTime = rshift<2>(triggerInfo.curGap) + rshift<3>(triggerInfo.curGap);
              break;
            default:
              triggerInfo.triggerFilterTime = 0;
              break;
          }
          break;

        case 1:
        case 3:
          // 70 degree tooth, next tooth is 170
          switch (configPage4.triggerFilter)
          {
            case 1: // 25 % 42 degrees
              triggerInfo.triggerFilterTime = rshift<1>(triggerInfo.curGap) + rshift<3>(triggerInfo.curGap);
              break;
            case 2: // 50 % 85 degrees
              triggerInfo.triggerFilterTime = triggerInfo.curGap + rshift<2>(triggerInfo.curGap);
              break;
            case 3: // 75 % 127 degrees
              triggerInfo.triggerFilterTime = triggerInfo.curGap + rshift<1>(triggerInfo.curGap) + rshift<2>(triggerInfo.curGap);
              break;
            default:
              triggerInfo.triggerFilterTime = 0;
              break;
          }
          break;

        default:
          triggerInfo.triggerFilterTime = 0;
          break;
      }

      //NEW IGNITION MODE
      if( (configPage2.perToothIgn == true) )
      {
        int16_t crankAngle = triggerInfo.toothAngles[triggerInfo.toothCurrentCount] + configPage4.triggerAngle;
        crankAngle = ignitionLimits(crankAngle);
        checkPerToothTiming(crankAngle, triggerInfo.toothCurrentCount);
      }

    } // has sync

  } //Trigger filter

}

void triggerSec_SuzukiK6A(void)
{
  return;
}

uint16_t getRPM_SuzukiK6A(void)
{
  //Cranking code needs working out.

  uint16_t tempRPM = stdGetRPM(CAM_SPEED);

  MAX_STALL_TIME = revolutionTime << 1; //Set the stall time to be twice the current RPM. This is a safe figure as there should be no single revolution where this changes more than this
  if(MAX_STALL_TIME < 366667UL) { MAX_STALL_TIME = 366667UL; } //Check for 50rpm minimum

  return tempRPM;
}

int getCrankAngle_SuzukiK6A(void)
{
  //Grab some variables that are used in the trigger code and assign them to temp variables.
  noInterrupts();
  uint16_t temptoothCurrentCount = triggerInfo.toothCurrentCount;
  unsigned long temptoothLastToothTime = triggerInfo.toothLastToothTime;
  triggerInfo.lastCrankAngleCalc = micros(); //micros() is no longer interrupt safe
  interrupts();

  if (temptoothCurrentCount>0U) {
    triggerInfo.triggerToothAngle = (uint16_t)triggerInfo.toothAngles[temptoothCurrentCount] - (uint16_t)triggerInfo.toothAngles[temptoothCurrentCount-1U];
  }

  //Estimate the number of degrees travelled since the last tooth}
  triggerInfo.elapsedTime = (triggerInfo.lastCrankAngleCalc - temptoothLastToothTime);

  int crankAngle = triggerInfo.toothAngles[temptoothCurrentCount] + configPage4.triggerAngle; //Perform a lookup of the fixed triggerInfo.toothAngles array to find what the angle of the last tooth passed was.
  crankAngle += (int)timeToAngleDegPerMicroSec(triggerInfo.elapsedTime);
  if (crankAngle >= 720) { crankAngle -= 720; }
  if (crankAngle < 0) { crankAngle += 720; }

  return crankAngle;
}

// Assumes no advance greater than 48 degrees. Triggers on the tooth before the ignition event
static uint16_t __attribute__((noinline)) calcEndTeeth_SuzukiK6A(int ignitionAngle) {
  //Temp variables are used here to avoid potential issues if a trigger interrupt occurs part way through this function
  const int16_t tempIgnitionEndTooth = ignitionLimits(ignitionAngle - configPage4.triggerAngle);

  uint8_t nCount=1U;
  while ((nCount<8U) && (tempIgnitionEndTooth > triggerInfo.toothAngles[nCount])) {
    ++nCount;
  }
  if(nCount==1U || nCount==8U) {
    // didn't find a match, use tooth 7 as it must be greater than 7 but less than 1.
    return 7U;
  }

  // The tooth we want is the tooth prior to this one.
  return nCount-1U;
}

void triggerSetEndTeeth_SuzukiK6A(void)
{
  triggerInfo.ignition1EndTooth = calcEndTeeth_SuzukiK6A(ignition1EndAngle);
  triggerInfo.ignition2EndTooth = calcEndTeeth_SuzukiK6A(ignition2EndAngle);
  triggerInfo.ignition3EndTooth = calcEndTeeth_SuzukiK6A(ignition3EndAngle);
}

/** @} */

