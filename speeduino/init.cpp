/** @file
 * Speeduino Initialisation (called at Arduino setup()).
 */
#include "globals.h"
#include "init.h"
#include "engine.h"
#include "storage.h"
#include "config.h"
#include "updates.h"
#include "speeduino.h"
#include "timers.h"
#include "comms.h"
#include "comms_secondary.h"
#include "comms_CAN.h"
#include "utilities.h"
#include "scheduledIO.h"
#include "scheduler.h"
#include "schedule_calcs.h"
#include "auxiliaries.h"
#include "sensors.h"
#include "decoders.h"
#include "corrections.h"
#include "idle.h"
#include "tables/table2d.h"
#include "acc_mc33810.h"


#if defined(EEPROM_RESET_PIN)
  #include EEPROM_LIB_H
#endif
#ifdef SD_LOGGING
  #include "SD_logger.h"
  #include "rtc_common.h"
#endif

#if defined(CORE_AVR)
#pragma GCC push_options
// This minimizes RAM usage at no performance cost
#pragma GCC optimize ("Os") 
#endif

#if !defined(UNIT_TEST)
static inline 
#endif
void construct2dTables(void) {
  //Repoint the 2D table structs to the config pages that were just loaded
  construct2dTable(taeTable,                  _countof(configPage4.taeValues),                  configPage4.taeValues,                  configPage4.taeBins);
  construct2dTable(maeTable,                  _countof(configPage4.maeRates),                   configPage4.maeRates,                   configPage4.maeBins);
  construct2dTable(WUETable,                  _countof(configPage2.wueValues),                  configPage2.wueValues,                  configPage4.wueBins);
  construct2dTable(ASETable,                  _countof(configPage2.asePct),                     configPage2.asePct,                     configPage2.aseBins);
  construct2dTable(ASECountTable,             _countof(configPage2.aseCount),                   configPage2.aseCount,                   configPage2.aseBins);
  construct2dTable(PrimingPulseTable,         _countof(configPage2.primePulse),                 configPage2.primePulse,                 configPage2.primeBins);
  construct2dTable(crankingEnrichTable,       _countof(configPage10.crankingEnrichValues),      configPage10.crankingEnrichValues,      configPage10.crankingEnrichBins);
  construct2dTable(dwellVCorrectionTable,     _countof(configPage4.dwellCorrectionValues),      configPage4.dwellCorrectionValues,      configPage6.voltageCorrectionBins);
  construct2dTable(injectorVCorrectionTable,  _countof(configPage6.injVoltageCorrectionValues), configPage6.injVoltageCorrectionValues, configPage6.voltageCorrectionBins);
  construct2dTable(IATDensityCorrectionTable, _countof(configPage6.airDenRates),                configPage6.airDenRates,                configPage6.airDenBins);
  construct2dTable(baroFuelTable,             _countof(configPage4.baroFuelValues),             configPage4.baroFuelValues,             configPage4.baroFuelBins);
  construct2dTable(IATRetardTable,            _countof(configPage4.iatRetValues),               configPage4.iatRetValues,               configPage4.iatRetBins);
  construct2dTable(CLTAdvanceTable,           _countof(configPage4.cltAdvValues),               configPage4.cltAdvValues,               configPage4.cltAdvBins);
  construct2dTable(idleTargetTable,           _countof(configPage6.iacCLValues),                configPage6.iacCLValues,                configPage6.iacBins);
  construct2dTable(idleAdvanceTable,          _countof(configPage4.idleAdvValues),              configPage4.idleAdvValues,              configPage4.idleAdvBins);
  construct2dTable(rotarySplitTable,          _countof(configPage10.rotarySplitValues),         configPage10.rotarySplitValues,         configPage10.rotarySplitBins);
  construct2dTable(flexFuelTable,             _countof(configPage10.flexFuelAdj),               configPage10.flexFuelAdj,               configPage10.flexFuelBins);
  construct2dTable(flexAdvTable,              _countof(configPage10.flexAdvAdj),                configPage10.flexAdvAdj,                configPage10.flexAdvBins);
  construct2dTable(fuelTempTable,             _countof(configPage10.fuelTempValues),            configPage10.fuelTempValues,            configPage10.fuelTempBins);
  construct2dTable(oilPressureProtectTable,   _countof(configPage10.oilPressureProtMins),       configPage10.oilPressureProtMins,       configPage10.oilPressureProtRPM);
  construct2dTable(coolantProtectTable,       _countof(configPage9.coolantProtRPM),             configPage9.coolantProtRPM,             configPage9.coolantProtTemp);
  construct2dTable(fanPWMTable,               _countof(configPage9.PWMFanDuty),                 configPage9.PWMFanDuty,                 configPage6.fanPWMBins);
  construct2dTable(wmiAdvTable,               _countof(configPage10.wmiAdvAdj),                 configPage10.wmiAdvAdj,                 configPage10.wmiAdvBins);
  construct2dTable(rollingCutTable,           _countof(configPage15.rollingProtCutPercent),     configPage15.rollingProtCutPercent,     configPage15.rollingProtRPMDelta);
  construct2dTable(injectorAngleTable,        _countof(configPage2.injAng),                     configPage2.injAng,                     configPage2.injAngRPM);
  construct2dTable(flexBoostTable,            _countof(configPage10.flexBoostAdj),              configPage10.flexBoostAdj,              configPage10.flexBoostBins);
  construct2dTable(knockWindowStartTable,      _countof(configPage10.knock_window_angle),        configPage10.knock_window_angle, configPage10.knock_window_rpms);
  construct2dTable(knockWindowDurationTable,   _countof(configPage10.knock_window_dur),          configPage10.knock_window_dur,   configPage10.knock_window_rpms);
  construct2dTable(cltCalibrationTable,       _countof(cltCalibration_values), cltCalibration_values, cltCalibration_bins);
  construct2dTable(iatCalibrationTable,       _countof(iatCalibration_values), iatCalibration_values, iatCalibration_bins);
  construct2dTable(o2CalibrationTable,        _countof(o2Calibration_values),  o2Calibration_values,  o2Calibration_bins);
}

/** Initialise Speeduino for the main loop.
 * Top level init entry point for all initialisations:
 * - Initialise and set sizes of 3D tables
 * - Load config from EEPROM, update config structures to current version of SW if needed.
 * - Initialise board (The initBoard() is for board X implemented in board_X.ino file)
 * - Initialise timers (See timers.ino)
 * - Perform optional SD card and RTC battery inits
 * - Load calibration tables from EEPROM
 * - Perform pin mapping (calling @ref setPinMapping() based on @ref config2.pinMapping)
 * - Stop any coil charging and close injectors
 * - Initialise schedulers, Idle, Fan, auxPWM, Corrections, AD-conversions, Programmable I/O
 * - Initialise baro (ambient pressure) by reading MAP (before engine runs)
 * - Initialise triggers (by @ref initialiseTriggers() )
 * - Perform cyl. count based initialisations (@ref config2.nCylinders)
 * - Perform injection and spark mode based setup
 *   - Assign injector open/close and coil charge begin/end functions to their dedicated global vars
 * - Perform fuel pressure priming by turning fuel pump on
 * - Read CLT and TPS sensors to have cranking pulsewidths computed correctly
 * - Mark Initialisation completed (this flag-marking is used in code to prevent after-init changes)
 */
void initialiseAll(void)
{   
    currentStatus.fpPrimed = false;
    currentStatus.injPrimed = false;

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    setupBoardSystemPinNum();		// setup system pin


    #if defined(CORE_STM32)
    configPage9.intcan_available = 1;   // device has internal canbus
    //STM32 can not currently enabled
    #endif

    /*
    ***********************************************************************************************************
    * EEPROM reset
    */
    #if defined(EEPROM_RESET_PIN) && !defined(UNIT_TEST)
    uint32_t start_time = millis();
    byte exit_erase_loop = false; 
    pinMode(EEPROM_RESET_PIN, INPUT_PULLUP);  

    //only start routine when this pin is low because it is pulled low
    while (digitalRead(EEPROM_RESET_PIN) != HIGH && (millis() - start_time)<1050)
    {
      //make sure the key is pressed for at least 0.5 second 
      if ((millis() - start_time)>500) {
        //if key is pressed afterboot for 0.5 second make led turn off
        digitalWrite(LED_BUILTIN, HIGH);

        //see if the user reacts to the led turned off with removing the keypress within 1 second
        while (((millis() - start_time)<1000) && (exit_erase_loop!=true)){

          //if user let go of key within 1 second erase eeprom
          if(digitalRead(EEPROM_RESET_PIN) != LOW){
            #if defined(FLASH_AS_EEPROM_h)
              EEPROM.read(0); //needed for SPI eeprom emulation.
              EEPROM.clear(); 
            #else 
              for (int i = 0 ; i < EEPROM.length() ; i++) { EEPROM.write(i, 255);}
            #endif
            //if erase done exit while loop.
            exit_erase_loop = true;
          }
        }
      } 
    }
    #endif
  

    // Unit tests should be independent of any stored configuration on the board!
#if !defined(UNIT_TEST)
    loadConfig();
    doUpdates(); //Check if any data items need updating (Occurs with firmware updates)
#endif


    //Always start with a clean slate on the bootloader capabilities level
    //This should be 0 until we hear otherwise from the 16u2
    configPage4.bootloaderCaps = 0;
    
    initBoard(); //This calls the current individual boards init function. See the board_xxx.ino files for these.

    initialiseTimers();
    
  #ifdef SD_LOGGING
    initRTC();
    if(configPage13.onboard_log_file_style) { initSD(); }
  #endif

//Teensy 4.1 does not require .begin() to be called. This introduces a 700ms delay on startup time whilst USB is enumerated if it is called
#ifndef CORE_TEENSY41
    Serial.begin(115200);
#endif

    #if !defined(CORE_M451)
    pPrimarySerial = &Serial; //Default to standard Serial interface
    							// no ptr available
	#endif

    BIT_SET(currentStatus.status4, BIT_STATUS4_ALLOW_LEGACY_COMMS); //Flag legacy comms as being allowed on startup

    //Repoint the 2D table structs to the config pages that were just loaded
    construct2dTables();
    
    //Setup the calibration tables
    loadCalibration();   

    //Set the pin mappings
    if((configPage2.pinMapping == 255) || (configPage2.pinMapping == 0)) //255 = EEPROM value in a blank AVR; 0 = EEPROM value in new FRAM
    {
      //First time running on this board
      resetConfigPages();
      setPinMapping(3); //Force board to v0.4
    }
    else
    {
    	setPinMapping(configPage2.pinMapping);
    }

    // Repeatedly initialising the CAN bus hangs the system when
    // running initialisation tests on Teensy 3.5
    #if defined(NATIVE_CAN_AVAILABLE) && !defined(UNIT_TEST)
      initCAN();
    #endif

    //Must come after setPinMapping() as secondary serial can be changed on a per board basis
    #if defined(secondarySerial_AVAILABLE)
      if (configPage9.enable_secondarySerial == 1) { secondarySerial.begin(115200); }
    #endif

    //End all coil charges to ensure no stray sparks on startup
    endCoil1Charge();
    endCoil2Charge();
    endCoil3Charge();
    endCoil4Charge();
    endCoil5Charge();
    #if (IGN_CHANNELS >= 6)
    endCoil6Charge();
    #endif
    #if (IGN_CHANNELS >= 7)
    endCoil7Charge();
    #endif
    #if (IGN_CHANNELS >= 8)
    endCoil8Charge();
    #endif

    //Similar for injectors, make sure they're turned off
    closeInjector1();
    closeInjector2();
    closeInjector3();
    closeInjector4();
    closeInjector5();
    #if (INJ_CHANNELS >= 6)
    closeInjector6();
    #endif
    #if (INJ_CHANNELS >= 7)
    closeInjector7();
    #endif
    #if (INJ_CHANNELS >= 8)
    closeInjector8();
    #endif
    
    //Set the tacho output default state
    digitalWrite(pinTachOut, HIGH);

    //Perform all initialisations
    initialiseSchedulers();

    //initialiseDisplay();
    initialiseIdle(true);
    initialiseFan();
    initialiseAirCon();
    initialiseAuxPWM();
    initialiseCorrections();

    BIT_CLEAR(currentStatus.engineProtectStatus, PROTECT_IO_ERROR); //Clear the I/O error bit. The bit will be set in initialiseADC() if there is problem in there.

    initialiseADC();
    initialiseMAPBaro();
    initialiseProgrammableIO();

    //Check whether the flex sensor is enabled and if so, attach an interrupt for it
    if(configPage2.flexEnabled > 0)
    {
      if(!pinIsReserved(pinFlex)) { attachInterrupt(digitalPinToInterrupt(pinFlex), flexPulse, CHANGE); }
      currentStatus.ethanolPct = 0;
    }
    //Same as above, but for the VSS input
    if(configPage2.vssMode > 1) // VSS modes 2 and 3 are interrupt drive (Mode 1 is CAN)
    {
      if(!pinIsReserved(pinVSS)) { attachInterrupt(digitalPinToInterrupt(pinVSS), vssPulse, RISING); }
    }
    //As above but for knock pulses
    if(configPage10.knock_mode == KNOCK_MODE_DIGITAL)
    {
      if(configPage10.knock_pullup) { pinMode(configPage10.knock_pin, INPUT_PULLUP); }
      else { pinMode(configPage10.knock_pin, INPUT); }

      if(!pinIsReserved(configPage10.knock_pin)) 
      { 
        if(configPage10.knock_trigger == KNOCK_TRIGGER_HIGH) { attachInterrupt(digitalPinToInterrupt(configPage10.knock_pin), knockPulse, RISING); }
        else { attachInterrupt(digitalPinToInterrupt(configPage10.knock_pin), knockPulse, FALLING); }
      }
    }

    //Once the configs have been loaded, a number of one time calculations can be completed
    req_fuel_uS = configPage2.reqFuel * 100; //Convert to uS and an int. This is the only variable to be used in calculations
    inj_opentime_uS = configPage2.injOpen * 100; //Injector open time. Comes through as ms*10 (Eg 15.5ms = 155).

    if(configPage10.stagingEnabled == true)
    {
    uint32_t totalInjector = configPage10.stagedInjSizePri + configPage10.stagedInjSizeSec;
    /*
        These values are a percentage of the req_fuel value that would be required for each injector channel to deliver that much fuel.
        Eg:
        Pri injectors are 250cc
        Sec injectors are 500cc
        Total injector capacity = 750cc

        staged_req_fuel_mult_pri = 300% (The primary injectors would have to run 3x the overall PW in order to be the equivalent of the full 750cc capacity
        staged_req_fuel_mult_sec = 150% (The secondary injectors would have to run 1.5x the overall PW in order to be the equivalent of the full 750cc capacity
    */
    staged_req_fuel_mult_pri = (100 * totalInjector) / configPage10.stagedInjSizePri;
    staged_req_fuel_mult_sec = (100 * totalInjector) / configPage10.stagedInjSizeSec;
    }

    if (configPage4.trigPatternSec == SEC_TRIGGER_POLL && configPage4.TrigPattern == DECODER_MISSING_TOOTH)
    { configPage4.TrigEdgeSec = configPage4.PollLevelPolarity; } // set the secondary trigger edge automatically to correct working value with poll level mode to enable cam angle detection in closed loop vvt.
    //Explanation: currently cam trigger for VVT is only captured when revolution one == 1. So we need to make sure that the edge trigger happens on the first revolution. So now when we set the poll level to be low
    //on revolution one and it's checked at tooth #1. This means that the cam signal needs to go high during the first revolution to be high on next revolution at tooth #1. So poll level low = cam trigger edge rising.

    //Begin the main crank trigger interrupt pin setup
    //The interrupt numbering is a bit odd - See here for reference: arduino.cc/en/Reference/AttachInterrupt
    //These assignments are based on the Arduino Mega AND VARY BETWEEN BOARDS. Please confirm the board you are using and update accordingly.
    currentStatus.RPM = 0;
    currentStatus.hasSync = false;
    BIT_CLEAR(currentStatus.status3, BIT_STATUS3_HALFSYNC);
    currentStatus.runSecs = 0;
    currentStatus.secl = 0;
    //currentStatus.seclx10 = 0;
    currentStatus.startRevolutions = 0;
    currentStatus.syncLossCounter = 0;
    currentStatus.flatShiftingHard = false;
    currentStatus.launchingHard = false;
    currentStatus.crankRPM = ((unsigned int)configPage4.crankRPM * 10); //Crank RPM limit (Saves us calculating this over and over again. It's updated once per second in timers.ino)
    currentStatus.fuelPumpOn = false;
    currentStatus.engineProtectStatus = 0;
    triggerInfo.triggerFilterTime = 0; //Trigger filter time is the shortest possible time (in uS) that there can be between crank teeth (ie at max RPM). Any pulses that occur faster than this time will be discarded as noise. This is simply a default value, the actual values are set in the setup() functions of each decoder
    dwellLimit_uS = (1000 * configPage4.dwellLimit);
    currentStatus.nChannels = ((uint8_t)INJ_CHANNELS << 4) + IGN_CHANNELS; //First 4 bits store the number of injection channels, 2nd 4 store the number of ignition channels
    fpPrimeTime = 0;
    ms_counter = 0;
    fixedCrankingOverride = 0;
    timer5_overflow_count = 0;
    toothHistoryIndex = 0;

    resetDecoder();
    
    noInterrupts();
    initialiseTriggers();

    //The secondary input can be used for VSS if nothing else requires it. Allows for the standard VR conditioner to be used for VSS. This MUST be run after the initialiseTriggers() function
    if( VSS_USES_RPM2() )
    {
    	attachInterrupt(digitalPinToInterrupt(pinVSS), vssPulse, RISING);
    } //Secondary trigger input can safely be used for VSS

    if( FLEX_USES_RPM2() )
    {
    	attachInterrupt(digitalPinToInterrupt(pinFlex), flexPulse, CHANGE);
    } //Secondary trigger input can safely be used for Flex sensor

    //End crank trigger interrupt attachment
    if(configPage2.strokes == FOUR_STROKE)
    {
      //Default is 1 squirt per revolution, so we halve the given req-fuel figure (Which would be over 2 revolutions)
      req_fuel_uS = req_fuel_uS / 2; //The req_fuel calculation above gives the total required fuel (At VE 100%) in the full cycle. If we're doing more than 1 squirt per cycle then we need to split the amount accordingly. (Note that in a non-sequential 4-stroke setup you cannot have less than 2 squirts as you cannot determine the stroke to make the single squirt on)
    }

    //Initial values for loop times
    currentLoopTime = micros_safe();
    mainLoopCount = 0;

    engineInit();

    //Begin priming the fuel pump. This is turned off in the low resolution, 1s interrupt in timers.ino
    //First check that the priming time is not 0
    if(configPage2.fpPrime > 0)
    {
      FUEL_PUMP_ON();
      currentStatus.fuelPumpOn = true;
    }
    else
    {
    	currentStatus.fpPrimed = true;
    } //If the user has set 0 for the pump priming, immediately mark the priming as being completed

    interrupts();

    readCLT(false); // Need to read coolant temp to make priming pulsewidth work correctly. The false here disables use of the filter
    readTPS(false); // Need to read tps to detect flood clear state

    /* tacho sweep function. */
    currentStatus.tachoSweepEnabled = (configPage2.useTachoSweep > 0);
    /* SweepMax is stored as a byte, RPM/100. divide by 60 to convert min to sec (net 5/3).  Multiply by ignition pulses per rev.
       tachoSweepIncr is also the number of tach pulses per second */
    tachoSweepIncr = configPage2.tachoSweepMaxRPM * maxIgnOutputs * 5 / 3;
    
    currentStatus.initialisationComplete = true;
    digitalWrite(LED_BUILTIN, HIGH);

}


/** Set board / microcontroller specific pin mappings / assignments.
 * The boardID is switch-case compared against raw boardID integers (not enum or defined label, and probably no need for that either)
 * which are originated from tuning SW (e.g. TS) set values and are available in reference/speeduino.ini (See pinLayout, note also that
 * numbering is not contiguous here).
 */
void setPinMapping(byte boardID)
{
  //Force set defaults. Will be overwritten below if needed.
#if !defined(CORE_M451)
  injectorOutputControl = OUTPUT_CONTROL_DIRECT;
  ignitionOutputControl = OUTPUT_CONTROL_DIRECT;
#endif

  if( configPage4.triggerTeeth == 0 )
  {
	  configPage4.triggerTeeth = 4;
  } //Avoid potential divide by 0 when starting decoders


  setupBoardFixedPinNum(boardID);		// first setup for fixed function pins

  setupRemappablePinNum(boardID);		// follow setup for remappable function pins (simple gpio ?)



  //Finally, set the relevant pin modes for outputs
  pinMode(pinTachOut, OUTPUT);
  pinMode(pinIdle1, OUTPUT);
  pinMode(pinIdle2, OUTPUT);
  pinMode(pinIdleUpOutput, OUTPUT);
  pinMode(pinFuelPump, OUTPUT);
  pinMode(pinFan, OUTPUT);
  pinMode(pinStepperDir, OUTPUT);
  pinMode(pinStepperStep, OUTPUT);
  pinMode(pinStepperEnable, OUTPUT);
  pinMode(pinBoost, OUTPUT);
  pinMode(pinVVT_1, OUTPUT);
  pinMode(pinVVT_2, OUTPUT);

  if(configPage4.ignBypassEnabled > 0)
  {
	  pinMode(pinIgnBypass, OUTPUT);
  }

  //This is a legacy mode option to revert the MAP reading behaviour to match what was in place prior to the 201905 firmware
  if(configPage2.legacyMAP > 0)
  {
	  digitalWrite(pinMAP, HIGH);
  }

#if !defined(CORE_M451)
  if( ignitionOutputControl == OUTPUT_CONTROL_DIRECT )
  {
#endif
	  pinMode(pinCoil1, OUTPUT);
    pinMode(pinCoil2, OUTPUT);
    pinMode(pinCoil3, OUTPUT);
    pinMode(pinCoil4, OUTPUT);
    #if (IGN_CHANNELS >= 5)
    pinMode(pinCoil5, OUTPUT);
    #endif
    #if (IGN_CHANNELS >= 6)
    pinMode(pinCoil6, OUTPUT);
    #endif
    #if (IGN_CHANNELS >= 7)
    pinMode(pinCoil7, OUTPUT);
    #endif
    #if (IGN_CHANNELS >= 8)
    pinMode(pinCoil8, OUTPUT);
    #endif

#if !defined(CORE_M451)

  } 

  if(injectorOutputControl == OUTPUT_CONTROL_DIRECT)
  {
#endif
    pinMode(pinInjector1, OUTPUT);
    pinMode(pinInjector2, OUTPUT);
    pinMode(pinInjector3, OUTPUT);
    pinMode(pinInjector4, OUTPUT);
    #if (INJ_CHANNELS >= 5)
    pinMode(pinInjector5, OUTPUT);
    #endif
    #if (INJ_CHANNELS >= 6)
    pinMode(pinInjector6, OUTPUT);
    #endif
    #if (INJ_CHANNELS >= 7)
    pinMode(pinInjector7, OUTPUT);
    #endif
    #if (INJ_CHANNELS >= 8)
    pinMode(pinInjector8, OUTPUT);
    #endif

#if defined(PORT_TYPE)
    inj1_pin_port = portOutputRegister(digitalPinToPort(pinInjector1));
    inj1_pin_mask = digitalPinToBitMask(pinInjector1);
    inj2_pin_port = portOutputRegister(digitalPinToPort(pinInjector2));
    inj2_pin_mask = digitalPinToBitMask(pinInjector2);
    inj3_pin_port = portOutputRegister(digitalPinToPort(pinInjector3));
    inj3_pin_mask = digitalPinToBitMask(pinInjector3);
    inj4_pin_port = portOutputRegister(digitalPinToPort(pinInjector4));
    inj4_pin_mask = digitalPinToBitMask(pinInjector4);
    inj5_pin_port = portOutputRegister(digitalPinToPort(pinInjector5));
    inj5_pin_mask = digitalPinToBitMask(pinInjector5);
    inj6_pin_port = portOutputRegister(digitalPinToPort(pinInjector6));
    inj6_pin_mask = digitalPinToBitMask(pinInjector6);
    inj7_pin_port = portOutputRegister(digitalPinToPort(pinInjector7));
    inj7_pin_mask = digitalPinToBitMask(pinInjector7);
    inj8_pin_port = portOutputRegister(digitalPinToPort(pinInjector8));
    inj8_pin_mask = digitalPinToBitMask(pinInjector8);
#endif

#if !defined(CORE_M451)
  }
#endif
  
#ifdef USE_ACCEL_MC_33810
  if( (ignitionOutputControl == OUTPUT_CONTROL_MC33810) || (injectorOutputControl == OUTPUT_CONTROL_MC33810) )
  {
    initMC33810();
    if( (LED_BUILTIN != SCK) && (LED_BUILTIN != MOSI) && (LED_BUILTIN != MISO) ) pinMode(LED_BUILTIN, OUTPUT); //This is required on as the LED pin can otherwise be reset to an input
  }
#endif

//CS pin number is now set in a compile flag. 
// #ifdef USE_SPI_EEPROM
//   //We need to send the flash CS (SS) pin if we're using SPI flash. It cannot read from globals.
//   EEPROM.begin(USE_SPI_EEPROM);
// #endif

#if defined(PORT_TYPE)
  tach_pin_port = portOutputRegister(digitalPinToPort(pinTachOut));
  tach_pin_mask = digitalPinToBitMask(pinTachOut);
  pump_pin_port = portOutputRegister(digitalPinToPort(pinFuelPump));
  pump_pin_mask = digitalPinToBitMask(pinFuelPump);
#endif


  //Each of the below are only set when their relevant function is enabled. This can help prevent pin conflicts that users aren't aware of with unused functions
  if( (configPage2.flexEnabled > 0) && (!pinIsOutput(pinFlex)) )
  {
    pinMode(pinFlex, INPUT); //Standard GM / Continental flex sensor requires pullup, but this should be onboard. The internal pullup will not work (Requires ~3.3k)!
  }
  if( (configPage2.vssMode > 1) && (!pinIsOutput(pinVSS)) ) //Pin mode 1 for VSS is CAN
  {
    pinMode(pinVSS, INPUT);
  }
  if( (configPage6.launchEnabled > 0) && (!pinIsOutput(pinLaunch)) )
  {
    if (configPage6.lnchPullRes == true) { pinMode(pinLaunch, INPUT_PULLUP); }
    else { pinMode(pinLaunch, INPUT); } //If Launch Pull Resistor is not set make input float.
  }
  if( (configPage2.idleUpEnabled > 0) && (!pinIsOutput(pinIdleUp)) )
  {
    if (configPage2.idleUpPolarity == 0) { pinMode(pinIdleUp, INPUT_PULLUP); } //Normal setting
    else { pinMode(pinIdleUp, INPUT); } //inverted setting
  }
  if( (configPage2.CTPSEnabled > 0) && (!pinIsOutput(pinCTPS)) )
  {
    if (configPage2.CTPSPolarity == 0) { pinMode(pinCTPS, INPUT_PULLUP); } //Normal setting
    else { pinMode(pinCTPS, INPUT); } //inverted setting
  }
  if( (configPage10.fuel2Mode == FUEL2_MODE_INPUT_SWITCH) && (!pinIsOutput(pinFuel2Input)) )
  {
    if (configPage10.fuel2InputPullup == true) { pinMode(pinFuel2Input, INPUT_PULLUP); } //With pullup
    else { pinMode(pinFuel2Input, INPUT); } //Normal input
  }

  if( (configPage10.spark2Mode == SPARK2_MODE_INPUT_SWITCH) && (!pinIsOutput(pinSpark2Input)) )
  {
    if (configPage10.spark2InputPullup == true) { pinMode(pinSpark2Input, INPUT_PULLUP); } //With pullup
    else { pinMode(pinSpark2Input, INPUT); } //Normal input
  }

  if( (configPage10.fuelPressureEnable > 0)  && (!pinIsOutput(pinFuelPressure)) )
  {
    pinMode(pinFuelPressure, INPUT);
  }

  if( (configPage10.oilPressureEnable > 0) && (!pinIsOutput(pinOilPressure)) )
  {
    pinMode(pinOilPressure, INPUT);
  }

  #ifdef SD_LOGGING
  if( (configPage13.onboard_log_trigger_Epin > 0) && (!pinIsOutput(pinSDEnable)) )
  {
    pinMode(pinSDEnable, INPUT);
  }
#endif

  if(configPage10.wmiEnabled > 0)
  {
    pinMode(pinWMIEnabled, OUTPUT);
    if(configPage10.wmiIndicatorEnabled > 0)
    {
      pinMode(pinWMIIndicator, OUTPUT);
      if (configPage10.wmiIndicatorPolarity > 0) { digitalWrite(pinWMIIndicator, HIGH); }
    }
    if( (configPage10.wmiEmptyEnabled > 0) && (!pinIsOutput(pinWMIEmpty)) )
    {
      if (configPage10.wmiEmptyPolarity == 0) { pinMode(pinWMIEmpty, INPUT_PULLUP); } //Normal setting
      else { pinMode(pinWMIEmpty, INPUT); } //inverted setting
    }
  } 

#if !defined(CORE_M451)
  if((pinAirConComp>0) && ((configPage15.airConEnable) == 1))
  {
    pinMode(pinAirConComp, OUTPUT);
  }

  if((pinAirConRequest > 0) && ((configPage15.airConEnable) == 1) && (!pinIsOutput(pinAirConRequest)))
  {
    if((configPage15.airConReqPol) == 1)
    {
      // Inverted
      // +5V is ON, Use external pull-down resistor for OFF
      pinMode(pinAirConRequest, INPUT);
    }
    else
    {
      //Normal
      // Pin pulled to Ground is ON. Floating (internally pulled up to +5V) is OFF.
      pinMode(pinAirConRequest, INPUT_PULLUP);
    }
  }

  if((pinAirConFan > 0) && ((configPage15.airConEnable) == 1) && ((configPage15.airConFanEnabled) == 1))
  {
    pinMode(pinAirConFan, OUTPUT);
  }  

#endif

#if defined(PORT_TYPE)
  //These must come after the above pinMode statements
  triggerPri_pin_port = portInputRegister(digitalPinToPort(pinTrigger));
  triggerPri_pin_mask = digitalPinToBitMask(pinTrigger);
  triggerSec_pin_port = portInputRegister(digitalPinToPort(pinTrigger2));
  triggerSec_pin_mask = digitalPinToBitMask(pinTrigger2);
  triggerThird_pin_port = portInputRegister(digitalPinToPort(pinTrigger3));
  triggerThird_pin_mask = digitalPinToBitMask(pinTrigger3);

  flex_pin_port = portInputRegister(digitalPinToPort(pinFlex));
  flex_pin_mask = digitalPinToBitMask(pinFlex);
#endif

}

static inline bool isAnyFuelScheduleRunning(void) {
  return fuelSchedule1.Status==RUNNING
      || fuelSchedule2.Status==RUNNING
      || fuelSchedule3.Status==RUNNING
      || fuelSchedule4.Status==RUNNING
#if INJ_CHANNELS >= 5      
      || fuelSchedule5.Status==RUNNING
#endif
#if INJ_CHANNELS >= 6
      || fuelSchedule6.Status==RUNNING
#endif
#if INJ_CHANNELS >= 7
      || fuelSchedule7.Status==RUNNING
#endif
#if INJ_CHANNELS >= 8
      || fuelSchedule8.Status==RUNNING
#endif
      ;
}

static inline bool isAnyIgnScheduleRunning(void) {
  return ignitionSchedule1.Status==RUNNING      
#if IGN_CHANNELS >= 2 
      || ignitionSchedule2.Status==RUNNING
#endif      
#if IGN_CHANNELS >= 3 
      || ignitionSchedule3.Status==RUNNING
#endif      
#if IGN_CHANNELS >= 4       
      || ignitionSchedule4.Status==RUNNING
#endif      
#if IGN_CHANNELS >= 5      
      || ignitionSchedule5.Status==RUNNING
#endif
#if IGN_CHANNELS >= 6
      || ignitionSchedule6.Status==RUNNING
#endif
#if IGN_CHANNELS >= 7
      || ignitionSchedule7.Status==RUNNING
#endif
#if IGN_CHANNELS >= 8
      || ignitionSchedule8.Status==RUNNING
#endif
      ;
}

/** Change injectors or/and ignition angles to 720deg.
 * Roll back req_fuel size and set number of outputs equal to cylinder count.
* */
void changeHalfToFullSync(void)
{
  //Need to do another check for injLayout as this function can be called from ignition
  noInterrupts();
  if( (configPage2.injLayout == INJ_SEQUENTIAL) && (CRANK_ANGLE_MAX_INJ != 720) && (!isAnyFuelScheduleRunning()))
  {
    CRANK_ANGLE_MAX_INJ = 720;
    req_fuel_uS *= 2;
    
    fuelSchedule1.pStartFunction = openInjector1;
    fuelSchedule1.pEndFunction = closeInjector1;
    fuelSchedule2.pStartFunction = openInjector2;
    fuelSchedule2.pEndFunction = closeInjector2;
    fuelSchedule3.pStartFunction = openInjector3;
    fuelSchedule3.pEndFunction = closeInjector3;
    fuelSchedule4.pStartFunction = openInjector4;
    fuelSchedule4.pEndFunction = closeInjector4;
#if INJ_CHANNELS >= 5
    fuelSchedule5.pStartFunction = openInjector5;
    fuelSchedule5.pEndFunction = closeInjector5;
#endif
#if INJ_CHANNELS >= 6
    fuelSchedule6.pStartFunction = openInjector6;
    fuelSchedule6.pEndFunction = closeInjector6;
#endif
#if INJ_CHANNELS >= 7
    fuelSchedule7.pStartFunction = openInjector7;
    fuelSchedule7.pEndFunction = closeInjector7;
#endif
#if INJ_CHANNELS >= 8
    fuelSchedule8.pStartFunction = openInjector8;
     fuelSchedule8.pEndFunction = closeInjector8;
#endif

    switch (configPage2.nCylinders)
    {
      case 4:
        maxInjOutputs = 4;
        break;
            
      case 6:
        maxInjOutputs = 6;
        break;

      case 8:
        maxInjOutputs = 8;
        break;

      default:
        break; //No actions required for other cylinder counts

    }
  }
  interrupts();

  //Need to do another check for sparkMode as this function can be called from injection
  if( (configPage4.sparkMode == IGN_MODE_SEQUENTIAL) && (CRANK_ANGLE_MAX_IGN != 720) && (!isAnyIgnScheduleRunning()) )
  {
    CRANK_ANGLE_MAX_IGN = 720;
    maxIgnOutputs = configPage2.nCylinders;
    switch (configPage2.nCylinders)
    {
    case 4:
      ignitionSchedule1.pStartCallback = beginCoil1Charge;
      ignitionSchedule1.pEndCallback = endCoil1Charge;
      ignitionSchedule2.pStartCallback = beginCoil2Charge;
      ignitionSchedule2.pEndCallback = endCoil2Charge;
      break;

    case 6:
      ignitionSchedule1.pStartCallback = beginCoil1Charge;
      ignitionSchedule1.pEndCallback = endCoil1Charge;
      ignitionSchedule2.pStartCallback = beginCoil2Charge;
      ignitionSchedule2.pEndCallback = endCoil2Charge;
      ignitionSchedule3.pStartCallback = beginCoil3Charge;
      ignitionSchedule3.pEndCallback = endCoil3Charge;
      break;

    case 8:
      ignitionSchedule1.pStartCallback = beginCoil1Charge;
      ignitionSchedule1.pEndCallback = endCoil1Charge;
      ignitionSchedule2.pStartCallback = beginCoil2Charge;
      ignitionSchedule2.pEndCallback = endCoil2Charge;
      ignitionSchedule3.pStartCallback = beginCoil3Charge;
      ignitionSchedule3.pEndCallback = endCoil3Charge;
      ignitionSchedule4.pStartCallback = beginCoil4Charge;
      ignitionSchedule4.pEndCallback = endCoil4Charge;
      break;

    default:
      break; //No actions required for other cylinder counts
      
    }
  }
}

/** Change injectors or/and ignition angles to 360deg.
 * In semi sequentiol mode req_fuel size is half.
 * Set number of outputs equal to half cylinder count.
* */
void changeFullToHalfSync(void)
{
  if(configPage2.injLayout == INJ_SEQUENTIAL)
  {
    CRANK_ANGLE_MAX_INJ = 360;
    req_fuel_uS /= 2;
    switch (configPage2.nCylinders)
    {
      case 4:
        if(configPage4.inj4cylPairing == INJ_PAIR_13_24)
        {
          fuelSchedule1.pStartFunction = openInjector1and3;
          fuelSchedule1.pEndFunction = closeInjector1and3;
          fuelSchedule2.pStartFunction = openInjector2and4;
          fuelSchedule2.pEndFunction = closeInjector2and4;
        }
        else
        {
          fuelSchedule1.pStartFunction = openInjector1and4;
          fuelSchedule1.pEndFunction = closeInjector1and4;
          fuelSchedule2.pStartFunction = openInjector2and3;
          fuelSchedule2.pEndFunction = closeInjector2and3;
        }
        maxInjOutputs = 2;
        break;
            
      case 6:
        fuelSchedule1.pStartFunction = openInjector1and4;
        fuelSchedule1.pEndFunction = closeInjector1and4;
        fuelSchedule2.pStartFunction = openInjector2and5;
        fuelSchedule2.pEndFunction = closeInjector2and5;
        fuelSchedule3.pStartFunction = openInjector3and6;
        fuelSchedule3.pEndFunction = closeInjector3and6;
        maxInjOutputs = 3;
        break;

      case 8:
        fuelSchedule1.pStartFunction = openInjector1and5;
        fuelSchedule1.pEndFunction = closeInjector1and5;
        fuelSchedule2.pStartFunction = openInjector2and6;
        fuelSchedule2.pEndFunction = closeInjector2and6;
        fuelSchedule3.pStartFunction = openInjector3and7;
        fuelSchedule3.pEndFunction = closeInjector3and7;
        fuelSchedule4.pStartFunction = openInjector4and8;
        fuelSchedule4.pEndFunction = closeInjector4and8;
        maxInjOutputs = 4;
        break;
    }
  }

  if(configPage4.sparkMode == IGN_MODE_SEQUENTIAL)
  {
    CRANK_ANGLE_MAX_IGN = 360;
    maxIgnOutputs = configPage2.nCylinders / 2;
    switch (configPage2.nCylinders)
    {
      case 4:
        ignitionSchedule1.pStartCallback = beginCoil1and3Charge;
        ignitionSchedule1.pEndCallback = endCoil1and3Charge;
        ignitionSchedule2.pStartCallback = beginCoil2and4Charge;
        ignitionSchedule2.pEndCallback = endCoil2and4Charge;
        break;
            
      case 6:
        ignitionSchedule1.pStartCallback = beginCoil1and4Charge;
        ignitionSchedule1.pEndCallback = endCoil1and4Charge;
        ignitionSchedule2.pStartCallback = beginCoil2and5Charge;
        ignitionSchedule2.pEndCallback = endCoil2and5Charge;
        ignitionSchedule3.pStartCallback = beginCoil3and6Charge;
        ignitionSchedule3.pEndCallback = endCoil3and6Charge;
        break;

      case 8:
        ignitionSchedule1.pStartCallback = beginCoil1and5Charge;
        ignitionSchedule1.pEndCallback = endCoil1and5Charge;
        ignitionSchedule2.pStartCallback = beginCoil2and6Charge;
        ignitionSchedule2.pEndCallback = endCoil2and6Charge;
        ignitionSchedule3.pStartCallback = beginCoil3and7Charge;
        ignitionSchedule3.pEndCallback = endCoil3and7Charge;
        ignitionSchedule4.pStartCallback = beginCoil4and8Charge;
        ignitionSchedule4.pEndCallback = endCoil4and8Charge;
        break;
    }
  }
}

#if defined(CORE_AVR)
#pragma GCC pop_options
#endif
