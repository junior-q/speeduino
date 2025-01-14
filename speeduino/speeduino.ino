/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,la
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/
/** @file
 * Speeduino initialisation and main loop.
 */
#include <stdint.h> //developer.mbed.org/handbook/C-Data-Types
//************************************************
#include <SimplyAtomic.h>
#include "globals.h"

#include "config.h"
#include "storage.h"

#include "speeduino.h"
#include "scheduler.h"
#include "comms.h"
#include "comms_legacy.h"
#include "comms_secondary.h"
#include "pages.h"

#include "TSComms.h"

#include "maths.h"
#include "corrections.h"
#include "timers.h"
#include "decoders.h"
#include "idle.h"
#include "auxiliaries.h"
#include "sensors.h"
#include "crankMaths.h"
#include "init.h"
#include "utilities.h"
#include "engineProtection.h"
#include "engine.h"
#include "scheduledIO.h"
#include "secondaryTables.h"
#include "comms_CAN.h"
#include "SD_logger.h"
#include "schedule_calcs.h"
#include "auxiliaries.h"
//#include BOARD_H //Note that this is not a real file, it is defined in globals.h.
//#include RTC_LIB_H //Defined in each boards .h file


static void updateIdleTarget(void);
static void auxControl(void);
static void serialControl(void);
static void checkLaunchAndFlatShift(void);
static void wmiLamp(void);		// No water indicator bulb

byte loopTimerMask;			// current loop tick timer mask
uint32_t currentLoopTime; /**< The time (in uS) that the current mainloop started */

const byte pinLedDemo = 38;


//#ifndef UNIT_TEST // Scope guard for unit testing
void setup(void)
{
	Spi.begin();				// SPI0 -> connect to a flash mem
	Spi1.begin();				// SPI1 -> connect to L9779
	Spi2.begin();				// SPI2 -> connect to Baro & FRAM

	pinMode(pinLedDemo, OUTPUT);

	currentStatus.initialisationComplete = false; //Tracks whether the initialiseAll() function has run completely
  initialiseAll();
}


/** Speeduino main loop.
 * 
 * Main loop chores (roughly in the order that they are performed):
 * - Check if serial comms or tooth logging are in progress (send or receive, prioritise communication)
 * - Record loop timing vars
 * - Check tooth time, update @ref statuses (currentStatus) variables
 * - Read sensors
 * - get VE for fuel calcs and spark advance for ignition
 * - Check crank/cam/tooth/timing sync (skip remaining ops if out-of-sync)
 * - execute doCrankSpeedCalcs()
 * 
 * single byte variable @ref LOOP_TIMER plays a big part here as:
 * - it contains expire-bits for interval based frequency driven events (e.g. 15Hz, 4Hz, 1Hz)
 * - Can be tested for certain frequency interval being expired by (eg) BIT_CHECK(LOOP_TIMER, BIT_TIMER_15HZ)
 * 
 */
//#pragma GCC diagnostic push
//#pragma GCC diagnostic ignored "-Wattributes"
// Sometimes loop() is inlined by LTO & sometimes not
// When not inlined, there is a huge difference in stack usage: 60+ bytes
// That eats into available RAM.
// Adding __attribute__((always_inline)) forces the LTO process to inline.
//
// Since the function is declared in an Arduino header, we can't change
// it to inline, so we need to suppress the resulting warning.
//void __attribute__((always_inline)) loop(void)

long count;


void loop(void)
{
	if(mainLoopCount < UINT16_MAX) { mainLoopCount++; }

    count++;

    if( count >= 10000 )
    {
  	    count = 0;

  	    digitalWrite(pinLedDemo, !digitalRead(pinLedDemo));

        SYS_UnlockReg();
        WDT_RESET_COUNTER();
        SYS_LockReg();
    }



	ATOMIC(){
		loopTimerMask = TIMER_mask;		// make a running copy of timer tick mask
		TIMER_mask = 0;					// reset isr routine mask
	}

    currentLoopTime = micros_safe();		// register current loop time

    //SERIAL Comms
	serialControl();

	Storage.storageControl();			// control machine for mem storage

    if( (configPage6.iacAlgorithm == IAC_ALGORITHM_STEP_OL)
    || (configPage6.iacAlgorithm == IAC_ALGORITHM_STEP_CL)
    || (configPage6.iacAlgorithm == IAC_ALGORITHM_STEP_OLCL) )
    {
      idleControl(); //Run idlecontrol every loop for stepper idle.
    }

    //***Perform sensor reads***
    //-----------------------------------------------------------------------------------------------------
    // Every 1ms. NOTE: This is NOT guaranteed to run at 1kHz on AVR systems. It will run at 1kHz if possible or as fast as loops/s allows if not.
    if (BIT_CHECK(loopTimerMask, BIT_TIMER_1KHZ))
    {
      readMAP();
    }

    if(BIT_CHECK(loopTimerMask, BIT_TIMER_200HZ))
    {
      #if defined(ANALOG_ISR)
        //ADC in free running mode does 1 complete conversion of all 16 channels and then the interrupt is disabled. Every 200Hz we re-enable the interrupt to get another conversion cycle
        BIT_SET(ADCSRA,ADIE); //Enable ADC interrupt
      #endif
    }

    if(	BIT_CHECK(loopTimerMask, BIT_TIMER_50HZ) ) //50 hertz
    {
      #if defined(NATIVE_CAN_AVAILABLE)
      sendCANBroadcast(50);
      #endif

      digitalUpdateL9979WD();
    }

    if(BIT_CHECK(loopTimerMask, BIT_TIMER_30HZ)) //30 hertz
    {

    	//Most boost tends to run at about 30Hz, so placing it here ensures a new target time is fetched frequently enough
      boostControl();

      //VVT may eventually need to be synced with the cam readings (ie run once per cam rev) but for now run at 30Hz
      vvtControl();

      //Water methanol injection
      wmiControl();

      #if TPS_READ_FREQUENCY == 30
        readTPS();
      #endif

      if (configPage2.canWBO == 0)
      {
        readO2();
        readO2_2();
      }
      
      #if defined(NATIVE_CAN_AVAILABLE)
      sendCANBroadcast(30);
      #endif

      #ifdef SD_LOGGING
        if(configPage13.onboard_log_file_rate == LOGGER_RATE_30HZ)
        {
        	writeSDLogEntry();
        }
      #endif

    }

    if (BIT_CHECK(loopTimerMask, BIT_TIMER_15HZ)) //Every 32 loops
    {
      #if TPS_READ_FREQUENCY == 15
        readTPS(); //TPS reading to be performed every 32 loops (any faster and it can upset the TPSdot sampling time)
      #endif

      #if  defined(CORE_TEENSY35)
          if (configPage9.enable_intcan == 1) // use internal can module
          {
           // this is just to test the interface is sending
           //sendCancommand(3,((configPage9.realtime_base_address & 0x3FF)+ 0x100),currentStatus.TPS,0,0x200);
          }
      #endif     

      checkLaunchAndFlatShift(); //Check for launch control and flat shift being active

      #if defined(NATIVE_CAN_AVAILABLE)
      sendCANBroadcast(15);
      #endif

      //And check whether the tooth log buffer is ready
      if( toothHistoryIndex > TOOTH_LOG_SIZE )
      {
    	  BIT_SET(currentStatus.status1, BIT_STATUS1_TOOTHLOG1READY);
      }
    }

    if(BIT_CHECK(loopTimerMask, BIT_TIMER_10HZ)) //10 hertz
    {
      //updateFullStatus();
      checkProgrammableIO();
      idleControl(); //Perform any idle related actions. This needs to be run at 10Hz to align with the idle taper resolution of 0.1s
      
      // Air conditioning control
      airConControl();

      currentStatus.vss = getSpeed();
      currentStatus.gear = getGear();

      #if defined(NATIVE_CAN_AVAILABLE)
      sendCANBroadcast(10);
      #endif

      #ifdef SD_LOGGING
        if(configPage13.onboard_log_file_rate == LOGGER_RATE_10HZ) { writeSDLogEntry(); }
      #endif

    }

    if (BIT_CHECK(loopTimerMask, BIT_TIMER_4HZ))
    {
      //The IAT and CLT readings can be done less frequently (4 times per second)
      readCLT();
      readIAT();
      readBat();

      nitrousControl();

      updateIdleTarget();

      #ifdef SD_LOGGING
        if(configPage13.onboard_log_file_rate == LOGGER_RATE_4HZ) { writeSDLogEntry(); }
        syncSDLog(); //Sync the SD log file to the card 4 times per second. 
      #endif  


      currentStatus.fuelPressure = getFuelPressure();
      currentStatus.oilPressure = getOilPressure();

      if(auxIsEnabled == true)
      {
          auxControl();
      } //aux channels are enabled

    } //4Hz timer

    if (BIT_CHECK(loopTimerMask, BIT_TIMER_1HZ)) //Once per second)
    {
      readBaro(); 		//Infrequent baro readings are not an issue.

      wmiLamp();		// No water indicator bulb

      #ifdef SD_LOGGING
        if(configPage13.onboard_log_file_rate == LOGGER_RATE_1HZ) { writeSDLogEntry(); }
      #endif

    } //1Hz timer


    engineControl();

#if !defined(CORE_M451)

    if ( (!BIT_CHECK(currentStatus.status3, BIT_STATUS3_RESET_PREVENT)) && (resetControl == RESET_CONTROL_PREVENT_WHEN_RUNNING) )
    {
        //Reset prevention is supposed to be on while the engine is running but isn't. Fix that.
        digitalWrite(pinResetControl, HIGH);
        BIT_SET(currentStatus.status3, BIT_STATUS3_RESET_PREVENT);
    } //Has sync and RPM
    else
    {
    	if ( (BIT_CHECK(currentStatus.status3, BIT_STATUS3_RESET_PREVENT) > 0) && (resetControl == RESET_CONTROL_PREVENT_WHEN_RUNNING) )
    	{
    		digitalWrite(pinResetControl, LOW);
    		BIT_CLEAR(currentStatus.status3, BIT_STATUS3_RESET_PREVENT);
    	}
    }
#endif

} //loop()
//#pragma GCC diagnostic pop

//#endif //Unit test guard


static void serialControl(void)
{

#if defined(CORE_M451)

	tsComm.serialTransmit();

	tsComm.serialReceive();

#else

  //SERIAL Comms
  //Initially check that the last serial send values request is not still outstanding
  if (serialTransmitInProgress())
  {
	serialTransmit();
  }

  //Check for any new or in-progress requests from serial.
  if (Serial.available()>0 || serialRecieveInProgress())
  {
	serialReceive();
  }

  //Check for any CAN comms requiring action
  #if defined(secondarySerial_AVAILABLE)
	//if can or secondary serial interface is enabled then check for requests.
	if (configPage9.enable_secondarySerial == 1)  //secondary serial interface enabled
	{
	  if ( ((mainLoopCount & 31) == 1) || (secondarySerial.available() > SERIAL_BUFFER_THRESHOLD) )
	  {
		if (secondarySerial.available() > 0)  { secondserial_Command(); }
	  }
	}
  #endif
  #if defined (NATIVE_CAN_AVAILABLE)
	if (configPage9.enable_intcan == 1) // use internal can module
	{
	  //check local can module
	  // if ( BIT_CHECK(LOOP_TIMER, BIT_TIMER_15HZ) or (CANbus0.available())
	  while (CAN_read())
	  {
		can_Command();
		readAuxCanBus();
		if (configPage2.canWBO > 0) { receiveCANwbo(); }
	  }
	}
  #endif
#endif
}


static void auxControl(void)
{
	//TODO dazq to clean this right up :)
	//check through the Aux input channels if enabled for Can or local use
	for (byte AuxinChan = 0; AuxinChan <16 ; AuxinChan++)
	{
		currentStatus.current_caninchannel = AuxinChan;

		if (((configPage9.caninput_sel[currentStatus.current_caninchannel]&12) == 4)
			&& (((configPage9.enable_secondarySerial == 1) && ((configPage9.enable_intcan == 0)&&(configPage9.intcan_available == 1)))
			|| ((configPage9.enable_secondarySerial == 1) && ((configPage9.enable_intcan == 1)&&(configPage9.intcan_available == 1))&&
			((configPage9.caninput_sel[currentStatus.current_caninchannel]&64) == 0))
			|| ((configPage9.enable_secondarySerial == 1) && ((configPage9.enable_intcan == 1)&&(configPage9.intcan_available == 0)))))
		{ //if current input channel is enabled as external & secondary serial enabled & internal can disabled(but internal can is available)
		  // or current input channel is enabled as external & secondary serial enabled & internal can enabled(and internal can is available)
		  //currentStatus.canin[13] = 11;  Dev test use only!
#ifdef SECONDARY_SERIAL_T	// only if used
		  if (configPage9.enable_secondarySerial == 1)  // megas only support can via secondary serial
		  {
			sendCancommand(2,0,currentStatus.current_caninchannel,0,((configPage9.caninput_source_can_address[currentStatus.current_caninchannel]&2047)+0x100));
			//send an R command for data from caninput_source_address[currentStatus.current_caninchannel] from secondarySerial
		  }
#endif
		}
		else
		{
			if (((configPage9.caninput_sel[currentStatus.current_caninchannel]&12) == 4)
			&& (((configPage9.enable_secondarySerial == 1) && ((configPage9.enable_intcan == 1)&&(configPage9.intcan_available == 1))&&
			((configPage9.caninput_sel[currentStatus.current_caninchannel]&64) == 64))
			|| ((configPage9.enable_secondarySerial == 0) && ((configPage9.enable_intcan == 1)&&(configPage9.intcan_available == 1))&&
			((configPage9.caninput_sel[currentStatus.current_caninchannel]&128) == 128))))
			{ //if current input channel is enabled as external for canbus & secondary serial enabled & internal can enabled(and internal can is available)
			  // or current input channel is enabled as external for canbus & secondary serial disabled & internal can enabled(and internal can is available)
			  //currentStatus.canin[13] = 12;  Dev test use only!
			#if defined(CORE_STM32) || defined(CORE_TEENSY)
			 if (configPage9.enable_intcan == 1) //  if internal can is enabled
			 {
				sendCancommand(3,configPage9.speeduino_tsCanId,currentStatus.current_caninchannel,0,((configPage9.caninput_source_can_address[currentStatus.current_caninchannel]&2047)+0x100));
				//send an R command for data from caninput_source_address[currentStatus.current_caninchannel] from internal canbus
			 }
			#endif
			}
			else
			{
				if ((((configPage9.enable_secondarySerial == 1) || ((configPage9.enable_intcan == 1) && (configPage9.intcan_available == 1))) && (configPage9.caninput_sel[currentStatus.current_caninchannel]&12) == 8)
				|| (((configPage9.enable_secondarySerial == 0) && ( (configPage9.enable_intcan == 1) && (configPage9.intcan_available == 0) )) && (configPage9.caninput_sel[currentStatus.current_caninchannel]&3) == 2)
				|| (((configPage9.enable_secondarySerial == 0) && (configPage9.enable_intcan == 0)) && ((configPage9.caninput_sel[currentStatus.current_caninchannel]&3) == 2)))
				{ //if current input channel is enabled as analog local pin
				  //read analog channel specified
				  //currentStatus.canin[13] = (configPage9.Auxinpina[currentStatus.current_caninchannel]&63);  Dev test use only!127
				  currentStatus.canin[currentStatus.current_caninchannel] = readAuxanalog(pinTranslateAnalog(configPage9.Auxinpina[currentStatus.current_caninchannel]&63));
				}
				else
				{
					if ((((configPage9.enable_secondarySerial == 1) || ((configPage9.enable_intcan == 1) && (configPage9.intcan_available == 1))) && (configPage9.caninput_sel[currentStatus.current_caninchannel]&12) == 12)
						|| (((configPage9.enable_secondarySerial == 0) && ( (configPage9.enable_intcan == 1) && (configPage9.intcan_available == 0) )) && (configPage9.caninput_sel[currentStatus.current_caninchannel]&3) == 3)
						|| (((configPage9.enable_secondarySerial == 0) && (configPage9.enable_intcan == 0)) && ((configPage9.caninput_sel[currentStatus.current_caninchannel]&3) == 3)))
					{ //if current input channel is enabled as digital local pin
					  //read digital channel specified
					  //currentStatus.canin[14] = ((configPage9.Auxinpinb[currentStatus.current_caninchannel]&63)+1);  Dev test use only!127+1
					  currentStatus.canin[currentStatus.current_caninchannel] = readAuxdigital((configPage9.Auxinpinb[currentStatus.current_caninchannel]&63)+1);
					} //Channel type
				}
			}
		}
	} //For loop going through each channel
}



static void updateIdleTarget(void)
{
	//Lookup the current target idle RPM. This is aligned with coolant and so needs to be calculated at the same rate CLT is read
	if( (configPage2.idleAdvEnabled >= 1) || (configPage6.iacAlgorithm != IAC_ALGORITHM_NONE) )
	{
	  currentStatus.CLIdleTarget = (byte)table2D_getValue(&idleTargetTable, currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET); //All temps are offset by 40 degrees
	  if(BIT_CHECK(currentStatus.airConStatus, BIT_AIRCON_TURNING_ON)) { currentStatus.CLIdleTarget += configPage15.airConIdleUpRPMAdder;  } //Adds Idle Up RPM amount if active
	}
}



static void checkLaunchAndFlatShift()
{
  //Check for launching/flat shift (clutch) based on the current and previous clutch states
  currentStatus.previousClutchTrigger = currentStatus.clutchTrigger;

  //Only check for pinLaunch if any function using it is enabled. Else pins might break starting a board
  if(configPage6.flatSEnable || configPage6.launchEnabled)
  {
    if(configPage6.launchHiLo > 0) { currentStatus.clutchTrigger = digitalRead(pinLaunch); }
    else { currentStatus.clutchTrigger = !digitalRead(pinLaunch); }
  }

  if(currentStatus.clutchTrigger && (currentStatus.previousClutchTrigger != currentStatus.clutchTrigger) ) { currentStatus.clutchEngagedRPM = currentStatus.RPM; } //Check whether the clutch has been engaged or disengaged and store the current RPM if so

  //Default flags to off
  currentStatus.launchingHard = false; 
  BIT_CLEAR(currentStatus.status2, BIT_STATUS2_HLAUNCH); 
  currentStatus.flatShiftingHard = false;

  if (configPage6.launchEnabled && currentStatus.clutchTrigger && (currentStatus.clutchEngagedRPM < ((unsigned int)(configPage6.flatSArm) * 100)) && (currentStatus.TPS >= configPage10.lnchCtrlTPS) ) 
  { 
    //If VSS is used, make sure we're not above the speed limit
    if ( (configPage2.vssMode > 0) && (currentStatus.vss >= configPage10.lnchCtrlVss) )
    {
      return;
    }
    
    //Check whether RPM is above the launch limit
    uint16_t launchRPMLimit = (configPage6.lnchHardLim * 100);
    if( (configPage2.hardCutType == HARD_CUT_ROLLING) ) { launchRPMLimit += (configPage15.rollingProtRPMDelta[0] * 10); } //Add the rolling cut delta if enabled (Delta is a negative value)

    if(currentStatus.RPM > launchRPMLimit)
    {
      //HardCut rev limit for 2-step launch control.
      currentStatus.launchingHard = true; 
      BIT_SET(currentStatus.status2, BIT_STATUS2_HLAUNCH); 
    }
  } 
  else 
  { 
    //If launch is not active, check whether flat shift should be active
    if(configPage6.flatSEnable && currentStatus.clutchTrigger && (currentStatus.clutchEngagedRPM >= ((unsigned int)(configPage6.flatSArm * 100)) ) ) 
    { 
      uint16_t flatRPMLimit = currentStatus.clutchEngagedRPM;
      if( (configPage2.hardCutType == HARD_CUT_ROLLING) ) { flatRPMLimit += (configPage15.rollingProtRPMDelta[0] * 10); } //Add the rolling cut delta if enabled (Delta is a negative value)

      if(currentStatus.RPM > flatRPMLimit)
      {
        //Flat shift rev limit
        currentStatus.flatShiftingHard = true;
      }
    }
  }
}


// No water indicator bulb
static void wmiLamp(void)
{
	if ( (configPage10.wmiEnabled > 0) && (configPage10.wmiIndicatorEnabled > 0) )
	{
	  // water tank empty
	  if (BIT_CHECK(currentStatus.status4, BIT_STATUS4_WMI_EMPTY) > 0)
	  {
		// flash with 1sec interval
		digitalWrite(pinWMIIndicator, !digitalRead(pinWMIIndicator));
	  }
	  else
	  {
		digitalWrite(pinWMIIndicator, configPage10.wmiIndicatorPolarity ? HIGH : LOW);
	  }
	}
}

//These function do checks on a pin to determine if it is already in use by another (higher importance) active function
bool pinIsOutput(byte pin)
{
  bool used = false;
  bool isIdlePWM = (configPage6.iacAlgorithm > 0) && ((configPage6.iacAlgorithm <= 3) || (configPage6.iacAlgorithm == 6));
  bool isIdleSteper = (configPage6.iacAlgorithm > 3) && (configPage6.iacAlgorithm != 6);
  //Injector?
  if ((pin == pinInjector1)
  || ((pin == pinInjector2) && (configPage2.nInjectors > 1))
  || ((pin == pinInjector3) && (configPage2.nInjectors > 2))
  || ((pin == pinInjector4) && (configPage2.nInjectors > 3))
#if (INJ_CHANNELS >= 5)
  || ((pin == pinInjector5) && (configPage2.nInjectors > 4))
  || ((pin == pinInjector6) && (configPage2.nInjectors > 5))
  || ((pin == pinInjector7) && (configPage2.nInjectors > 6))
  || ((pin == pinInjector8) && (configPage2.nInjectors > 7)))
#else
  )
#endif

  {
    used = true;
  }
  //Ignition?
  if ((pin == pinCoil1)
  || ((pin == pinCoil2) && (maxIgnOutputs > 1))
  || ((pin == pinCoil3) && (maxIgnOutputs > 2))
  || ((pin == pinCoil4) && (maxIgnOutputs > 3))
#if (INJ_CHANNELS >= 5)
  || ((pin == pinCoil5) && (maxIgnOutputs > 4))
  || ((pin == pinCoil6) && (maxIgnOutputs > 5))
  || ((pin == pinCoil7) && (maxIgnOutputs > 6))
  || ((pin == pinCoil8) && (maxIgnOutputs > 7)))
  {
    used = true;
  }
#else
  )
#endif

	//Functions?
  if ((pin == pinFuelPump)
  || ((pin == pinFan) && (configPage2.fanEnable == 1))
  || ((pin == pinVVT_1) && (configPage6.vvtEnabled > 0))
  || ((pin == pinVVT_2) && (configPage10.wmiEnabled > 0))
  || ((pin == pinVVT_2) && (configPage10.vvt2Enabled > 0))
  || ((pin == pinBoost) && (configPage6.boostEnabled == 1))
  || ((pin == pinIdle1) && isIdlePWM)
  || ((pin == pinIdle2) && isIdlePWM && (configPage6.iacChannels == 1))
  || ((pin == pinStepperEnable) && isIdleSteper)
  || ((pin == pinStepperStep) && isIdleSteper)
  || ((pin == pinStepperDir) && isIdleSteper)
  || (pin == pinTachOut)
#if !defined(CORE_M451)

  || ((pin == pinAirConComp) && (configPage15.airConEnable > 0))
  || ((pin == pinAirConFan) && (configPage15.airConEnable > 0) && (configPage15.airConFanEnabled > 0)) )
#else
	  )
#endif
  {
    used = true;
  }
  //Forbidden or hardware reserved? (Defined at board_xyz.h file)
  if ( pinIsReserved(pin) ) { used = true; }

  return used;
}

bool pinIsUsed(byte pin)
{
  bool used = false;

  //Analog input?
  if ( pinIsSensor(pin) )
  {
    used = true;
  }
  //Functions?
  if ( pinIsOutput(pin) )
  {
    used = true;
  }

  return used;
}

