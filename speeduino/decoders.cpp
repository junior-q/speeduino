#define MJR 1

/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart
A full copy of the license may be found in the projects root directory
*/

/** @file
 * 
 * Crank and Cam decoders
 * 
 * This file contains the various crank and cam wheel decoder functions.
 * Each decoder must have the following 4 functions (Where xxxx is the decoder name):
 * 
 * - **triggerSetup_xxxx** - Called once from within setup() and configures any required variables
 * - **triggerPri_xxxx** - Called each time the primary (No. 1) crank/cam signal is triggered (Called as an interrupt, so variables must be declared volatile)
 * - **triggerSec_xxxx** - Called each time the secondary (No. 2) crank/cam signal is triggered (Called as an interrupt, so variables must be declared volatile)
 * - **getRPM_xxxx** - Returns the current RPM, as calculated by the decoder
 * - **getCrankAngle_xxxx** - Returns the current crank angle, as calculated by the decoder
 * - **getCamAngle_xxxx** - Returns the current CAM angle, as calculated by the decoder
 *
 * Each decoder must utilise at least the following variables:
 * 
 * - triggerInfo.toothLastToothTime - The time (In uS) that the last primary tooth was 'seen'
 */

/* Notes on Doxygen Groups/Modules documentation style:
 * - Installing doxygen (e.g. Ubuntu) via pkg mgr: sudo apt-get install doxygen graphviz
 * - @defgroup tag name/description becomes the short name on (Doxygen) "Modules" page
 * - Relying on JAVADOC_AUTOBRIEF (in Doxyfile, essentially automatic @brief), the first sentence (ending with period) becomes
 *   the longer description (second column following name) on (Doxygen) "Modules" page (old Desc: ... could be this sentence)
 * - All the content after first sentence (like old Note:...) is visible on the page linked from the name (1st col) on "Modules" page
 * - To group all decoders together add 1) @defgroup dec Decoders (on top) and 2) "@ingroup dec" to each decoder (under @defgroup)
 * - To compare Speeduino Doxyfile to default config, do: `doxygen -g Doxyfile.default ; diff Doxyfile.default Doxyfile`
 */
#include <limits.h>
#include <SimplyAtomic.h>
#include "globals.h"
#include "config.h"

#include "decoders.h"
#include "scheduledIO.h"
#include "scheduler.h"
#include "crankMaths.h"
#include "timers.h"
#include "schedule_calcs.h"
#include "schedule_calcs.hpp"
#include "unit_testing.h"

//#include "decoders/decoder_missingTooth.h"

void nullTriggerHandler (void){return;} //initialisation function for triggerhandlers, does exactly nothing
uint16_t nullGetRPM(void){return 0;} //initialisation function for getRpm, returns safe value of 0
int nullGetCrankAngle(void){return 0;} //initialisation function for getCrankAngle, returns safe value of 0

static void triggerRoverMEMSCommon(void);
static inline void triggerRecordVVT1Angle (void);



void (*triggerHandler)(void) = nullTriggerHandler; ///Pointer for the trigger function (Gets pointed to the relevant decoder)
void (*triggerSecondaryHandler)(void) = nullTriggerHandler; ///Pointer for the secondary trigger function (Gets pointed to the relevant decoder)
void (*triggerTertiaryHandler)(void) = nullTriggerHandler; ///Pointer for the tertiary trigger function (Gets pointed to the relevant decoder)
uint16_t (*getRPM)(void) = nullGetRPM; ///Pointer to the getRPM function (Gets pointed to the relevant decoder)
int (*getCrankAngle)(void) = nullGetCrankAngle; ///Pointer to the getCrank Angle function (Gets pointed to the relevant decoder)
void (*triggerSetEndTeeth)(void) = triggerSetEndTeeth_missingTooth; ///Pointer to the triggerSetEndTeeth function of each decoder


uint32_t MAX_STALL_TIME = MICROS_PER_SEC/2U; 			//The maximum time (in uS) that the system will continue to function before the engine is considered stalled/stopped. This is unique to each decoder, depending on the number of teeth etc. 500000 (half a second) is used as the default value, most decoders will be much less.

triggerInfo_t	triggerInfo;


#ifdef USE_LIBDIVIDE
#include "src/libdivide/libdivide.h"
static libdivide::libdivide_s16_t divtriggerInfo.triggerToothAngle;
#endif

/** Universal (shared between decoders) decoder routines.
*
* @defgroup dec_uni Universal Decoder Routines
* 
* @{
*/
// whichTooth - 0 for Primary (Crank), 1 for Secondary (Cam)

/** Add tooth log entry to toothHistory (array).
 * Enabled by (either) currentStatus.toothLogEnabled and currentStatus.compositeTriggerUsed.
 * @param toothTime - Tooth Time
 * @param whichTooth - 0 for Primary (Crank), 2 for Secondary (Cam) 3 for Tertiary (Cam)
 */
static inline void addToothLogEntry(unsigned long toothTime, byte whichTooth)
{
  if(BIT_CHECK(currentStatus.status1, BIT_STATUS1_TOOTHLOG1READY)) { return; }
  //High speed tooth logging history
  if( (currentStatus.toothLogEnabled == true) || (currentStatus.compositeTriggerUsed > 0) ) 
  {
    bool valueLogged = false;
    if(currentStatus.toothLogEnabled == true)
    {
      //Tooth log only works on the Crank tooth
      if(whichTooth == TOOTH_CRANK)
      { 
        toothHistory[toothHistoryIndex] = toothTime; //Set the value in the log. 
        valueLogged = true;
      } 
    }
    else if(currentStatus.compositeTriggerUsed > 0)
    {
      compositeLogHistory[toothHistoryIndex] = 0;
      if(currentStatus.compositeTriggerUsed == 4)
      {
        // we want to display both cams so swap the values round to display primary as cam1 and secondary as cam2, include the crank in the data as the third output
        if(READ_SEC_TRIGGER() == true) { BIT_SET(compositeLogHistory[toothHistoryIndex], COMPOSITE_LOG_PRI); }
        if(READ_THIRD_TRIGGER() == true) { BIT_SET(compositeLogHistory[toothHistoryIndex], COMPOSITE_LOG_SEC); }
        if(READ_PRI_TRIGGER() == true) { BIT_SET(compositeLogHistory[toothHistoryIndex], COMPOSITE_LOG_THIRD); }
        if(whichTooth > TOOTH_CAM_SECONDARY) { BIT_SET(compositeLogHistory[toothHistoryIndex], COMPOSITE_LOG_TRIG); }
      }
      else
      {
        // we want to display crank and one of the cams
        if(READ_PRI_TRIGGER() == true) { BIT_SET(compositeLogHistory[toothHistoryIndex], COMPOSITE_LOG_PRI); }
        if(currentStatus.compositeTriggerUsed == 3)
        { 
          // display cam2 and also log data for cam 1
          if(READ_THIRD_TRIGGER() == true) { BIT_SET(compositeLogHistory[toothHistoryIndex], COMPOSITE_LOG_SEC); } // only the COMPOSITE_LOG_SEC value is visualised hence the swapping of the data
          if(READ_SEC_TRIGGER() == true) { BIT_SET(compositeLogHistory[toothHistoryIndex], COMPOSITE_LOG_THIRD); } 
        } 
        else
        { 
          // display cam1 and also log data for cam 2 - this is the historic composite view
          if(READ_SEC_TRIGGER() == true) { BIT_SET(compositeLogHistory[toothHistoryIndex], COMPOSITE_LOG_SEC); } 
          if(READ_THIRD_TRIGGER() == true) { BIT_SET(compositeLogHistory[toothHistoryIndex], COMPOSITE_LOG_THIRD); }
        }
        if(whichTooth > TOOTH_CRANK) { BIT_SET(compositeLogHistory[toothHistoryIndex], COMPOSITE_LOG_TRIG); }
      }  
      if(currentStatus.hasSync == true) { BIT_SET(compositeLogHistory[toothHistoryIndex], COMPOSITE_LOG_SYNC); }

      if(triggerInfo.revolutionOne == 1)
      { BIT_SET(compositeLogHistory[toothHistoryIndex], COMPOSITE_ENGINE_CYCLE);}
      else
      { BIT_CLEAR(compositeLogHistory[toothHistoryIndex], COMPOSITE_ENGINE_CYCLE);}

      toothHistory[toothHistoryIndex] = micros();
      valueLogged = true;
    }

    //If there has been a value logged above, update the indexes
    if(valueLogged == true)
    {
     if(toothHistoryIndex < (TOOTH_LOG_SIZE-1)) { toothHistoryIndex++; BIT_CLEAR(currentStatus.status1, BIT_STATUS1_TOOTHLOG1READY); }
     else { BIT_SET(currentStatus.status1, BIT_STATUS1_TOOTHLOG1READY); }
    }


  } //Tooth/Composite log enabled
}

/** Interrupt handler for primary trigger.
* This function is called on both the rising and falling edges of the primary trigger, when either the 
* composite or tooth loggers are turned on. 
*/
void loggerPrimaryISR(void)
{
  BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_VALID_TRIGGER); //This value will be set to the return value of the decoder function, indicating whether or not this pulse passed the filters
  bool validEdge = false; //This is set true below if the edge 
  /* 
  Need to still call the standard decoder trigger. 
  Two checks here:
  1) If the primary trigger is RISING, then check whether the primary is currently HIGH
  2) If the primary trigger is FALLING, then check whether the primary is currently LOW
  If either of these are true, the primary decoder function is called
  */
  if( ( (primaryTriggerEdge == RISING) && (READ_PRI_TRIGGER() == HIGH) ) || ( (primaryTriggerEdge == FALLING) && (READ_PRI_TRIGGER() == LOW) ) || (primaryTriggerEdge == CHANGE) )
  {
    triggerHandler();
    validEdge = true;
  }
  if( (currentStatus.toothLogEnabled == true) && (BIT_CHECK(triggerInfo.decoderState, BIT_DECODER_VALID_TRIGGER)) )
  {
    //Tooth logger only logs when the edge was correct
    if(validEdge == true) 
    { 
      addToothLogEntry(triggerInfo.curGap, TOOTH_CRANK);
    }
  }
  else if( (currentStatus.compositeTriggerUsed > 0) )
  {
    //Composite logger adds an entry regardless of which edge it was
    addToothLogEntry(triggerInfo.curGap, TOOTH_CRANK);
  }
}

/** Interrupt handler for secondary trigger.
* As loggerPrimaryISR, but for the secondary trigger.
*/
void loggerSecondaryISR(void)
{
  BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_VALID_TRIGGER); //This value will be set to the return value of the decoder function, indicating whether or not this pulse passed the filters
  BIT_SET(triggerInfo.decoderState, BIT_DECODER_VALID_TRIGGER); //This value will be set to the return value of the decoder function, indicating whether or not this pulse passed the filters
  /* 3 checks here:
  1) If the primary trigger is RISING, then check whether the primary is currently HIGH
  2) If the primary trigger is FALLING, then check whether the primary is currently LOW
  3) The secondary trigger is CHANGING
  If any of these are true, the primary decoder function is called
  */
  if( ( (secondaryTriggerEdge == RISING) && (READ_SEC_TRIGGER() == HIGH) ) || ( (secondaryTriggerEdge == FALLING) && (READ_SEC_TRIGGER() == LOW) ) || (secondaryTriggerEdge == CHANGE) )
  {
    triggerSecondaryHandler();
  }
  //No tooth logger for the secondary input
  if( (currentStatus.compositeTriggerUsed > 0) && (BIT_CHECK(triggerInfo.decoderState, BIT_DECODER_VALID_TRIGGER)) )
  {
    //Composite logger adds an entry regardless of which edge it was
    addToothLogEntry(triggerInfo.curGap2, TOOTH_CAM_SECONDARY);
  }
}

/** Interrupt handler for third trigger.
* As loggerPrimaryISR, but for the third trigger.
*/
void loggerTertiaryISR(void)
{
  BIT_CLEAR(triggerInfo.decoderState, BIT_DECODER_VALID_TRIGGER); //This value will be set to the return value of the decoder function, indicating whether or not this pulse passed the filters
  BIT_SET(triggerInfo.decoderState, BIT_DECODER_VALID_TRIGGER); //This value will be set to the return value of the decoder function, indicating whether or not this pulse passed the filters
  /* 3 checks here:
  1) If the primary trigger is RISING, then check whether the primary is currently HIGH
  2) If the primary trigger is FALLING, then check whether the primary is currently LOW
  3) The secondary trigger is CHANGING
  If any of these are true, the primary decoder function is called
  */
  
  
  if( ( (tertiaryTriggerEdge == RISING) && ( READ_THIRD_TRIGGER() == HIGH) ) || ( (tertiaryTriggerEdge == FALLING) && (READ_THIRD_TRIGGER() == LOW) ) || (tertiaryTriggerEdge == CHANGE) )
  {
    triggerTertiaryHandler();
  }
  //No tooth logger for the secondary input
  if( (currentStatus.compositeTriggerUsed > 0) && (BIT_CHECK(triggerInfo.decoderState, BIT_DECODER_VALID_TRIGGER)) )
  {
    //Composite logger adds an entry regardless of which edge it was
    addToothLogEntry(triggerInfo.curGap3, TOOTH_CAM_TERTIARY);
  }  
}

#if false
#if !defined(UNIT_TEST)
static
#endif
uint32_t angleToTimeIntervalTooth(uint16_t angle) {
  noInterrupts();
  if(BIT_CHECK(triggerInfo.decoderState, BIT_DECODER_TOOTH_ANG_CORRECT))
  {
    unsigned long toothTime = (triggerInfo.toothLastToothTime - triggerInfo.toothLastMinusOneToothTime);
    uint16_t temptriggerToothAngle = triggerInfo.triggerToothAngle; // triggerInfo.triggerToothAngle is set by interrupts
    interrupts();
    
    return (toothTime * (uint32_t)angle) / temptriggerToothAngle;
  }
  //Safety check. This can occur if the last tooth seen was outside the normal pattern etc
  else { 
    interrupts();
    return angleToTimeMicroSecPerDegree(angle); 
  }
}
#endif

static uint16_t timeToAngleIntervalTooth(uint32_t time)
{
    noInterrupts();
    //Still uses a last interval method (ie retrospective), but bases the interval on the gap between the 2 most recent teeth rather than the last full revolution
    if(BIT_CHECK(triggerInfo.decoderState, BIT_DECODER_TOOTH_ANG_CORRECT))
    {
      unsigned long toothTime = (triggerInfo.toothLastToothTime - triggerInfo.toothLastMinusOneToothTime);
      uint16_t temptriggerToothAngle = triggerInfo.triggerToothAngle; // triggerInfo.triggerToothAngle is set by interrupts
      interrupts();

      return (unsigned long)(time * (uint32_t)temptriggerToothAngle) / toothTime;
    }
    else { 
      interrupts();
      //Safety check. This can occur if the last tooth seen was outside the normal pattern etc
      return timeToAngleDegPerMicroSec(time);
    }
}

static inline bool IsCranking(const statuses &status) {
  return (status.RPM < status.crankRPM) && (status.startRevolutions == 0U);
}

bool engineIsRunning(uint32_t __curTime) {
  // Check how long ago the last tooth was seen compared to now. 
  // If it was more than MAX_STALL_TIME then the engine is probably stopped. 
  // triggerInfo.toothLastToothTime can be greater than triggerInfo.curTime if a pulse occurs between getting the latest time and doing the comparison
  ATOMIC() {
	  if( triggerInfo.toothLastToothTime > __curTime )
		  return true;

	  if( (__curTime - triggerInfo.toothLastToothTime) < MAX_STALL_TIME )
    	return true;
  }
  return false; // Just here to avoid compiler warning.
}

void resetDecoder(void) {
  triggerInfo.toothLastSecToothTime = 0;
  triggerInfo.toothLastToothTime = 0;
  triggerInfo.toothSystemCount = 0;
  triggerInfo.secondaryToothCount = 0;
}

#if defined(UNIT_TEST)
bool SetRevolutionTime(uint32_t revTime)
#else
static __attribute__((noinline)) bool SetRevolutionTime(uint32_t revTime)
#endif
{
  if (revTime!=revolutionTime) {
    revolutionTime = revTime;
    setAngleConverterRevolutionTime(revolutionTime);
    return true;
  } 
  return false;
}

static bool UpdateRevolutionTimeFromTeeth(bool isCamTeeth) {
  noInterrupts();
  bool updatedRevTime = HasAnySync(currentStatus) 
    && !IsCranking(currentStatus)
    && (triggerInfo.toothOneMinusOneTime!=UINT32_C(0))
    && (triggerInfo.toothOneTime>triggerInfo.toothOneMinusOneTime) 
    //The time in uS that one revolution would take at current speed (The time tooth 1 was last seen, minus the time it was seen prior to that)
    && SetRevolutionTime((triggerInfo.toothOneTime - triggerInfo.toothOneMinusOneTime) >> (isCamTeeth ? 1U : 0U)); 

  interrupts();
 return updatedRevTime;  
}

static inline uint16_t clampRpm(uint16_t rpm) {
    return rpm>=MAX_RPM ? currentStatus.RPM : rpm;
}

static inline uint16_t RpmFromRevolutionTimeUs(uint32_t revTime) {
  if (revTime<UINT16_MAX) {
    return clampRpm(udiv_32_16_closest(MICROS_PER_MIN, revTime));
  } else {
    return clampRpm((uint16_t)UDIV_ROUND_CLOSEST(MICROS_PER_MIN, revTime, uint32_t)); //Calc RPM based on last full revolution time (Faster as /)
  }
}

static inline uint16_t clampToToothCount(int16_t toothNum, uint8_t toothAdder) {
  int16_t toothRange = (int16_t)configPage4.triggerTeeth + (int16_t)toothAdder;
  return (uint16_t)nudge(1, toothRange, toothNum, toothRange);
}

static inline uint16_t clampToActualTeeth(uint16_t toothNum, uint8_t toothAdder) {
  if(toothNum > triggerInfo.triggerActualTeeth && toothNum <= configPage4.triggerTeeth) { toothNum = triggerInfo.triggerActualTeeth; }
  return min(toothNum, (uint16_t)(triggerInfo.triggerActualTeeth + toothAdder));
}


/** Compute RPM.
* As nearly all the decoders use a common method of determining RPM (The time the last full revolution took) A common function is simpler.
* @param degreesOver - the number of crank degrees between tooth #1s. Some patterns have a tooth #1 every crank rev, others are every cam rev.
* @return RPM
*/
static __attribute__((noinline)) uint16_t stdGetRPM(bool isCamTeeth)
{
  if (UpdateRevolutionTimeFromTeeth(isCamTeeth)) {
    return RpmFromRevolutionTimeUs(revolutionTime);
  }

  return currentStatus.RPM;
}

/**
 * Sets the new filter time based on the current settings.
 * This ONLY works for even spaced decoders.
 */
static void setFilter(unsigned long __curGap)
{
  /*
  if(configPage4.triggerFilter == 0) { triggerInfo.triggerFilterTime = 0; } //trigger filter is turned off.
  else if(configPage4.triggerFilter == 1) { triggerInfo.triggerFilterTime = triggerInfo.curGap >> 2; } //Lite filter level is 25% of previous gap
  else if(configPage4.triggerFilter == 2) { triggerInfo.triggerFilterTime = triggerInfo.curGap >> 1; } //Medium filter level is 50% of previous gap
  else if (configPage4.triggerFilter == 3) { triggerInfo.triggerFilterTime = (triggerInfo.curGap * 3) >> 2; } //Aggressive filter level is 75% of previous gap
  else { triggerInfo.triggerFilterTime = 0; } //trigger filter is turned off.
  */

  switch(configPage4.triggerFilter)
  {
    case TRIGGER_FILTER_OFF: 
      triggerInfo.triggerFilterTime = 0;
      break;
    case TRIGGER_FILTER_LITE: 
      triggerInfo.triggerFilterTime = __curGap >> 2;
      break;
    case TRIGGER_FILTER_MEDIUM: 
      triggerInfo.triggerFilterTime = __curGap >> 1;
      break;
    case TRIGGER_FILTER_AGGRESSIVE: 
      triggerInfo.triggerFilterTime = (__curGap * 3) >> 2;
      break;
    default:
      triggerInfo.triggerFilterTime = 0;
      break;
  }
}

/**
This is a special case of RPM measure that is based on the time between the last 2 teeth rather than the time of the last full revolution.
This gives much more volatile reading, but is quite useful during cranking, particularly on low resolution patterns.
It can only be used on patterns where the teeth are evenly spaced.
It takes an argument of the full (COMPLETE) number of teeth per revolution.
For a missing tooth wheel, this is the number if the tooth had NOT been missing (Eg 36-1 = 36)
*/
static __attribute__((noinline)) int crankingGetRPM(byte totalTeeth, bool isCamTeeth)
{
  if( (currentStatus.startRevolutions >= configPage4.StgCycles) && ((currentStatus.hasSync == true) || BIT_CHECK(currentStatus.status3, BIT_STATUS3_HALFSYNC)) )
  {
    if((triggerInfo.toothLastMinusOneToothTime > 0) && (triggerInfo.toothLastToothTime > triggerInfo.toothLastMinusOneToothTime) )
    {
      noInterrupts();
      bool newRevtime = SetRevolutionTime(((triggerInfo.toothLastToothTime - triggerInfo.toothLastMinusOneToothTime) * totalTeeth) >> (isCamTeeth ? 1U : 0U));
      interrupts();
      if (newRevtime) {
        return RpmFromRevolutionTimeUs(revolutionTime);
      }
    }
  }

  return currentStatus.RPM;
}

/**
On decoders that are enabled for per tooth based timing adjustments, this function performs the timer compare changes on the schedules themselves
For each ignition channel, a check is made whether we're at the relevant tooth and whether that ignition schedule is currently running
Only if both these conditions are met will the schedule be updated with the latest timing information.
If it's the correct tooth, but the schedule is not yet started, calculate and an end compare value (This situation occurs when both the start and end of the ignition pulse happen after the end tooth, but before the next tooth)
*/
static inline void checkPerToothTiming(int16_t crankAngle, uint16_t currentTooth)
{
  if ( (fixedCrankingOverride == 0) && (currentStatus.RPM > 0) )
  {
    if ( (currentTooth == triggerInfo.ignition1EndTooth) )
    {
      adjustCrankAngle(ignitionSchedule1, ignition1EndAngle, crankAngle);
    }
    else if ( (currentTooth == triggerInfo.ignition2EndTooth) )
    {
      adjustCrankAngle(ignitionSchedule2, ignition2EndAngle, crankAngle);
    }
    else if ( (currentTooth == triggerInfo.ignition3EndTooth) )
    {
      adjustCrankAngle(ignitionSchedule3, ignition3EndAngle, crankAngle);
    }
    else if ( (currentTooth == triggerInfo.ignition4EndTooth) )
    {
      adjustCrankAngle(ignitionSchedule4, ignition4EndAngle, crankAngle);
    }
#if IGN_CHANNELS >= 5
    else if ( (currentTooth == triggerInfo.ignition5EndTooth) )
    {
      adjustCrankAngle(ignitionSchedule5, ignition5EndAngle, crankAngle);
    }
#endif
#if IGN_CHANNELS >= 6
    else if ( (currentTooth == triggerInfo.ignition6EndTooth) )
    {
      adjustCrankAngle(ignitionSchedule6, ignition6EndAngle, crankAngle);
    }
#endif
#if IGN_CHANNELS >= 7
    else if ( (currentTooth == triggerInfo.ignition7EndTooth) )
    {
      adjustCrankAngle(ignitionSchedule7, ignition7EndAngle, crankAngle);
    }
#endif
#if IGN_CHANNELS >= 8
    else if ( (currentTooth == triggerInfo.ignition8EndTooth) )
    {
      adjustCrankAngle(ignitionSchedule8, ignition8EndAngle, crankAngle);
    }
#endif
  }
}
/** @} */
  

//************************************************************************************************************************

#include "decoders/decoder_24X.cxx"
#include "decoders/decoder_420a.cxx"
#include "decoders/decoder_4G63.cxx"
#include "decoders/decoder_Audi135.cxx"
#include "decoders/decoder_BasicDistributor.cxx"
#include "decoders/decoder_Daihatsu.cxx"
#include "decoders/decoder_DRZ400.cxx"
#include "decoders/decoder_DualWheel.cxx"
#include "decoders/decoder_FordST170.cxx"
#include "decoders/decoder_GM7X.cxx"
#include "decoders/decoder_Harley.cxx"
#include "decoders/decoder_HondaD17.cxx"
#include "decoders/decoder_HondaJ32.cxx"
#include "decoders/decoder_Jeep2000.cxx"
#include "decoders/decoder_MazdaAU.cxx"
#include "decoders/decoder_Miata9905.cxx"
#include "decoders/decoder_missingTooth.cxx"
#include "decoders/decoder_NGC.cxx"
#include "decoders/decoder_Nissan360.cxx"
#include "decoders/decoder_non360.cxx"
#include "decoders/decoder_Renix.cxx"
#include "decoders/decoder_RoverMEMS.cxx"
#include "decoders/decoder_Subaru67.cxx"
#include "decoders/decoder_SuzukiK6A.cxx"
#include "decoders/decoder_ThirtySixMinus21.cxx"
#include "decoders/decoder_ThirtySixMinus222.cxx"
#include "decoders/decoder_Vmax.cxx"
#include "decoders/decoder_Webber.cxx"


//************************************************************************************************************************
