/** @file
 * Instantiation of various (table2D, table3D) tables, volatile (interrupt modified) variables, Injector (1...8) enablement flags, etc.
 */
#include "globals.h"

const char TSfirmwareVersion[] PROGMEM = "Speeduino";


/// volatile inj*_pin_port and  inj*_pin_mask vars are for the direct port manipulation of the injectors, coils and aux outputs.
volatile PORT_TYPE *inj1_pin_port;
volatile PINMASK_TYPE inj1_pin_mask;
volatile PORT_TYPE *inj2_pin_port;
volatile PINMASK_TYPE inj2_pin_mask;
volatile PORT_TYPE *inj3_pin_port;
volatile PINMASK_TYPE inj3_pin_mask;
volatile PORT_TYPE *inj4_pin_port;
volatile PINMASK_TYPE inj4_pin_mask;
volatile PORT_TYPE *inj5_pin_port;
volatile PINMASK_TYPE inj5_pin_mask;
volatile PORT_TYPE *inj6_pin_port;
volatile PINMASK_TYPE inj6_pin_mask;
volatile PORT_TYPE *inj7_pin_port;
volatile PINMASK_TYPE inj7_pin_mask;
volatile PORT_TYPE *inj8_pin_port;
volatile PINMASK_TYPE inj8_pin_mask;

volatile PORT_TYPE *ign1_pin_port;
volatile PINMASK_TYPE ign1_pin_mask;
volatile PORT_TYPE *ign2_pin_port;
volatile PINMASK_TYPE ign2_pin_mask;
volatile PORT_TYPE *ign3_pin_port;
volatile PINMASK_TYPE ign3_pin_mask;
volatile PORT_TYPE *ign4_pin_port;
volatile PINMASK_TYPE ign4_pin_mask;
volatile PORT_TYPE *ign5_pin_port;
volatile PINMASK_TYPE ign5_pin_mask;
volatile PORT_TYPE *ign6_pin_port;
volatile PINMASK_TYPE ign6_pin_mask;
volatile PORT_TYPE *ign7_pin_port;
volatile PINMASK_TYPE ign7_pin_mask;
volatile PORT_TYPE *ign8_pin_port;
volatile PINMASK_TYPE ign8_pin_mask;

volatile PORT_TYPE *tach_pin_port;
volatile PINMASK_TYPE tach_pin_mask;
volatile PORT_TYPE *pump_pin_port;
volatile PINMASK_TYPE pump_pin_mask;

volatile PORT_TYPE *flex_pin_port;
volatile PINMASK_TYPE flex_pin_mask;

volatile PORT_TYPE *triggerPri_pin_port;
volatile PINMASK_TYPE triggerPri_pin_mask;
volatile PORT_TYPE *triggerSec_pin_port;
volatile PINMASK_TYPE triggerSec_pin_mask;
volatile PORT_TYPE *triggerThird_pin_port;
volatile PINMASK_TYPE triggerThird_pin_mask;

//These are variables used across multiple files
byte fpPrimeTime = 0; ///< The time (in seconds, based on @ref statuses.secl) that the fuel pump started priming
uint8_t softLimitTime = 0; //The time (in 0.1 seconds, based on seclx10) that the soft limiter started
volatile uint16_t mainLoopCount; //Main loop counter (incremented at each main loop rev., used for maintaining currentStatus.loopsPerSecond)
uint32_t revolutionTime; //The time in uS that one revolution would take at current speed (The time tooth 1 was last seen, minus the time it was seen prior to that)
volatile unsigned long timer5_overflow_count = 0; //Increments every time counter 5 overflows. Used for the fast version of micros()
volatile unsigned long ms_counter = 0; //A counter that increments once per ms
uint16_t fixedCrankingOverride = 0;
bool clutchTrigger;
bool previousClutchTrigger;
volatile uint32_t toothHistory[TOOTH_LOG_SIZE]; ///< Tooth trigger history - delta time (in uS) from last tooth (Indexed by @ref toothHistoryIndex)
volatile uint8_t compositeLogHistory[TOOTH_LOG_SIZE]; 
volatile unsigned int toothHistoryIndex = 0; ///< Current index to @ref toothHistory array
volatile uint16_t ignitionCount; /**< The count of ignition events that have taken place since the engine started */
#if defined(CORE_SAMD21)
  PinStatus primaryTriggerEdge;
  PinStatus secondaryTriggerEdge;
  PinStatus tertiaryTriggerEdge;
#else
  byte primaryTriggerEdge;
  byte secondaryTriggerEdge;
  byte tertiaryTriggerEdge;
#endif
int CRANK_ANGLE_MAX_IGN = 360;
int CRANK_ANGLE_MAX_INJ = 360; ///< The number of crank degrees that the system track over. Typically 720 divided by the number of squirts per cycle (Eg 360 for wasted 2 squirt and 720 for sequential single squirt)
volatile uint32_t runSecsX10;
volatile uint32_t seclx10;
volatile byte HWTest_INJ = 0; /**< Each bit in this variable represents one of the injector channels and it's HW test status */
volatile byte HWTest_INJ_Pulsed = 0; /**< Each bit in this variable represents one of the injector channels and it's pulsed HW test status */
volatile byte HWTest_IGN = 0; /**< Each bit in this variable represents one of the ignition channels and it's HW test status */
volatile byte HWTest_IGN_Pulsed = 0; 
byte maxIgnOutputs = 1; /**< Number of ignition outputs being used by the current tune configuration */
byte maxInjOutputs = 1; /**< Number of injection outputs being used by the current tune configuration */

//This needs to be here because using the config page directly can prevent burning the setting
byte resetControl = RESET_CONTROL_DISABLED;

volatile byte TIMER_mask;

/// Various pin numbering (Injectors, Ign outputs, CAS, Cam, Sensors. etc.) assignments
byte pinInjector1; ///< Output pin injector 1
byte pinInjector2; ///< Output pin injector 2
byte pinInjector3; ///< Output pin injector 3
byte pinInjector4; ///< Output pin injector 4
byte pinInjector5; ///< Output pin injector 5
byte pinInjector6; ///< Output pin injector 6
byte pinInjector7; ///< Output pin injector 7
byte pinInjector8; ///< Output pin injector 8
byte injectorOutputControl = OUTPUT_CONTROL_DIRECT; /**< Specifies whether the injectors are controlled directly (Via an IO pin)
    or using something like the MC33810. 0 = Direct (OUTPUT_CONTROL_DIRECT), 10 = MC33810 (OUTPUT_CONTROL_MC33810) */
byte pinCoil1; ///< Pin for coil 1
byte pinCoil2; ///< Pin for coil 2
byte pinCoil3; ///< Pin for coil 3
byte pinCoil4; ///< Pin for coil 4
byte pinCoil5; ///< Pin for coil 5
byte pinCoil6; ///< Pin for coil 6
byte pinCoil7; ///< Pin for coil 7
byte pinCoil8; ///< Pin for coil 8
byte ignitionOutputControl = OUTPUT_CONTROL_DIRECT; /**< Specifies whether the coils are controlled directly (Via an IO pin)
   or using something like the MC33810. 0 = Direct (OUTPUT_CONTROL_DIRECT), 10 = MC33810 (OUTPUT_CONTROL_MC33810) */
byte pinTrigger;  ///< RPM1 (Typically CAS=crankshaft angle sensor) pin
byte pinTrigger2; ///< RPM2 (Typically the Cam Sensor) pin
byte pinTrigger3;	///< the 2nd cam sensor pin
byte pinTPS;      //TPS input pin
byte pinMAP;      //MAP sensor pin
byte pinEMAP;     //EMAP sensor pin
byte pinMAP2;     //2nd MAP sensor (Currently unused)
byte pinIAT;      //IAT sensor pin
byte pinCLT;      //CLS sensor pin
byte pinO2;       //O2 Sensor pin
byte pinO2_2;     //second O2 pin
byte pinBat;      //Battery voltage pin
byte pinDisplayReset; // OLED reset pin
byte pinTachOut;  //Tacho output
byte pinFuelPump; //Fuel pump on/off
byte pinIdle1;    //Single wire idle control
byte pinIdle2;    //2 wire idle control (Not currently used)
byte pinIdleUp;   //Input for triggering Idle Up
byte pinIdleUpOutput; //Output that follows (normal or inverted) the idle up pin
byte pinCTPS;     //Input for triggering closed throttle state
byte pinFuel2Input;  //Input for switching to the 2nd fuel table
byte pinSpark2Input; //Input for switching to the 2nd ignition table
byte pinSpareTemp1;  // Future use only
byte pinSpareTemp2;  // Future use only
byte pinSpareOut1;  //Generic output
byte pinSpareOut2;  //Generic output
byte pinSpareOut3;  //Generic output
byte pinSpareOut4;  //Generic output
byte pinSpareOut5;  //Generic output
byte pinSpareOut6;  //Generic output
byte pinSpareHOut1; //spare high current output
byte pinSpareHOut2; // spare high current output
byte pinSpareLOut1; // spare low current output
byte pinSpareLOut2; // spare low current output
byte pinSpareLOut3;
byte pinSpareLOut4;
byte pinSpareLOut5;
byte pinBoost;
byte pinVVT_1;     ///< vvt (variable valve timing) output 1
byte pinVVT_2;     ///< vvt (variable valve timing) output 2
byte pinFan;       ///< Cooling fan output (on/off? See: auxiliaries.ino)
byte pinStepperDir; //Direction pin for the stepper motor driver
byte pinStepperStep; //Step pin for the stepper motor driver
byte pinStepperEnable; //Turning the DRV8825 driver on/off
byte pinLaunch;
byte pinIgnBypass; //The pin used for an ignition bypass (Optional)
byte pinFlex; //Pin with the flex sensor attached
byte pinVSS;  // VSS (Vehicle speed sensor) Pin
byte pinBaro; //Pin that an al barometric pressure sensor is attached to (If used)
byte pinResetControl; // Output pin used control resetting the Arduino
byte pinFuelPressure;
byte pinOilPressure;
byte pinWMIEmpty; // Water tank empty sensor
byte pinWMIIndicator; // No water indicator bulb
byte pinWMIEnabled; // ON-OFF output to relay/pump/solenoid 
byte pinMC33810_1_CS;
byte pinMC33810_2_CS;
byte pinSDEnable;
#ifdef USE_SPI_EEPROM
  byte pinSPIFlash_CS;
#endif
byte pinAirConComp;     // Air conditioning compressor output (See: auxiliaries.ino)
byte pinAirConFan;    // Stand-alone air conditioning fan output (See: auxiliaries.ino)
byte pinAirConRequest;  // Air conditioning request input (See: auxiliaries.ino)

struct statuses currentStatus; /**< The master global "live" status struct. Contains all values that are updated frequently and used across modules */
