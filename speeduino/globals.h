/** @file
 * Global defines, macros, struct definitions (@ref statuses, @ref config2, @ref config4, config*), extern-definitions (for globally accessible vars).
 * 
 * ### Note on configuration struct layouts
 * 
 * Once the struct members have been assigned to certain "role" (in certain SW version), they should not be "moved around"
 * as the structs are stored onto EEPROM as-is and the offset and size of member needs to remain constant. Also removing existing struct members
 * would disturb layouts. Because of this a certain amount unused old members will be left into the structs. For the storage related reasons also the
 * bit fields are defined in byte-size (or multiple of ...) chunks.
 * 
 * ### Config Structs and 2D, 3D Tables
 * 
 * The config* structures contain information coming from tuning SW (e.g. TS) for 2D and 3D tables, where looked up value is not a result of direct
 * array lookup, but from interpolation algorithm. Because of standard, reusable interpolation routines associated with structs table2D and table3D,
 * the values from config are copied from config* structs to table2D (table3D destined configurations are not stored in config* structures).
 * 
 * ### Board choice
 * There's a C-preprocessor based "#if defined" logic present in this header file based on the Arduino IDE compiler set CPU
 * (+board?) type, e.g. `__AVR_ATmega2560__`. This respectively drives (withi it's "#if defined ..." block):
 * - The setting of various BOARD_* C-preprocessor variables (e.g. BOARD_MAX_ADC_PINS)
 * - Setting of BOARD_H (Board header) file (e.g. "board_avr2560.h"), which is later used to include the header file
 *   - Seems Arduino ide implicitly compiles and links respective .ino file (by it's internal build/compilation rules) (?)
 * - Setting of CPU (?) CORE_* variables (e.g. CORE_AVR), that is used across codebase to distinguish CPU.
 */
#ifndef GLOBALS_H
#define GLOBALS_H
#include <Arduino.h>
#include <SimplyAtomic.h>
#include "tables/table2d.h"
#include "tables/table3d.h"
#include "src/FastCRC/FastCRC.h"

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
  #define BOARD_MAX_DIGITAL_PINS 54 //digital pins +1
  #define BOARD_MAX_IO_PINS 70 //digital pins + analog channels + 1
  #define BOARD_MAX_ADC_PINS  15 //Number of analog pins
  #ifndef LED_BUILTIN
    #define LED_BUILTIN 13
  #endif
  #define CORE_AVR
  #define BOARD_H "boards/board_avr2560.h"
  #ifndef INJ_CHANNELS
    #define INJ_CHANNELS 4
  #endif
  #ifndef IGN_CHANNELS
    #define IGN_CHANNELS 5
  #endif

  #if defined(__AVR_ATmega2561__)
    //This is a workaround to avoid having to change all the references to higher ADC channels. We simply define the channels (Which don't exist on the 2561) as being the same as A0-A7
    //These Analog inputs should never be used on any 2561 board definition (Because they don't exist on the MCU), so it will not cause any issues
    #define A8  A0
    #define A9  A1
    #define A10  A2
    #define A11  A3
    #define A12  A4
    #define A13  A5
    #define A14  A6
    #define A15  A7
  #endif

  //#define TIMER5_MICROS

#elif defined(CORE_TEENSY)
  #if defined(__MK64FX512__) || defined(__MK66FX1M0__)
    #define CORE_TEENSY35
    #define BOARD_H "boards/board_teensy35.h"
  #elif defined(__IMXRT1062__)
    #define CORE_TEENSY41
    #define BOARD_H "boards/board_teensy41.h"
  #endif
  #define INJ_CHANNELS 8
  #define IGN_CHANNELS 8

#elif defined(STM32_MCU_SERIES) || defined(ARDUINO_ARCH_STM32) || defined(STM32)
  #define BOARD_H "boards/board_stm32_official.h"
  #define CORE_STM32

  #define BOARD_MAX_ADC_PINS  NUM_ANALOG_INPUTS-1 //Number of analog pins from core.
  #if defined(STM32F407xx) //F407 can do 8x8 STM32F401/STM32F411 don't
   #define INJ_CHANNELS 8
   #define IGN_CHANNELS 8
  #else
   #define INJ_CHANNELS 4
   #define IGN_CHANNELS 5
  #endif
#elif defined(__SAMD21G18A__)
  #define BOARD_H "boards/board_samd21.h"
  #define CORE_SAMD21
  #define CORE_SAM
  #define INJ_CHANNELS 4
  #define IGN_CHANNELS 4
#elif defined(__SAMC21J18A__)
  #define BOARD_H "boards/board_samc21.h"
  #define CORE_SAMC21
  #define CORE_SAM
#elif defined(__SAME51J19A__)
  #define BOARD_H "boards/board_same51.h"
  #define CORE_SAME51
  #define CORE_SAM
  #define INJ_CHANNELS 8
  #define IGN_CHANNELS 8
#else
  #error Incorrect board selected. Please select the correct board (Usually Mega 2560) and upload again
#endif

//This can only be included after the above section
#include BOARD_H //Note that this is not a real file, it is defined in globals.h. 

//Handy bitsetting macros
#define BIT_SET(a,b) ((a) |= (1U<<(b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1U<<(b)))
#define BIT_CHECK(var,pos) !!((var) & (1U<<(pos)))
#define BIT_TOGGLE(var,pos) ((var)^= 1UL << (pos))
#define BIT_WRITE(var, pos, bitvalue) ((bitvalue) ? BIT_SET((var), (pos)) : bitClear((var), (pos)))

#define CRANK_ANGLE_MAX (max(CRANK_ANGLE_MAX_IGN, CRANK_ANGLE_MAX_INJ))

#define interruptSafe(c) (noInterrupts(); {c} interrupts();) //Wraps any code between nointerrupt and interrupt calls

#define MICROS_PER_SEC INT32_C(1000000)
#define MICROS_PER_MIN INT32_C(MICROS_PER_SEC*60U)
#define MICROS_PER_HOUR INT32_C(MICROS_PER_MIN*60U)

#define SERIAL_PORT_PRIMARY   0
#define SERIAL_PORT_SECONDARY 3

//Define the load algorithm
#define LOAD_SOURCE_MAP         0
#define LOAD_SOURCE_TPS         1
#define LOAD_SOURCE_IMAPEMAP    2

//Define bit positions within engine variable
#define BIT_ENGINE_RUN      0   // Engine running
#define BIT_ENGINE_CRANK    1   // Engine cranking
#define BIT_ENGINE_ASE      2   // after start enrichment (ASE)
#define BIT_ENGINE_WARMUP   3   // Engine in warmup
#define BIT_ENGINE_ACC      4   // in acceleration mode (TPS accel)
#define BIT_ENGINE_DCC      5   // in deceleration mode
#define BIT_ENGINE_MAPACC   6   // MAP acceleration mode
#define BIT_ENGINE_MAPDCC   7   // MAP deceleration mode

//Define masks for Status1
#define BIT_STATUS1_INJ1           0  //inj1
#define BIT_STATUS1_INJ2           1  //inj2
#define BIT_STATUS1_INJ3           2  //inj3
#define BIT_STATUS1_INJ4           3  //inj4
#define BIT_STATUS1_DFCO           4  //Deceleration fuel cutoff
#define BIT_STATUS1_BOOSTCUT       5  //Fuel component of MAP based boost cut out
#define BIT_STATUS1_TOOTHLOG1READY 6  //Used to flag if tooth log 1 is ready
#define BIT_STATUS1_TOOTHLOG2READY 7  //Used to flag if tooth log 2 is ready (Log is not currently used)

//Define masks for spark variable
#define BIT_STATUS2_HLAUNCH         0  //Hard Launch indicator
#define BIT_STATUS2_SLAUNCH         1  //Soft Launch indicator
#define BIT_STATUS2_HRDLIM          2  //Hard limiter indicator
#define BIT_STATUS2_SFTLIM          3  //Soft limiter indicator
#define BIT_STATUS2_BOOSTCUT        4  //Spark component of MAP based boost cut out
#define BIT_STATUS2_ERROR           5  // Error is detected
#define BIT_STATUS2_IDLE            6  // idle on
#define BIT_STATUS2_SYNC            7  // Whether engine has sync or not

#define BIT_STATUS5_FLATSH         0  //Flat shift hard cut
#define BIT_STATUS5_FLATSS         1  //Flat shift soft cut
#define BIT_STATUS5_SPARK2_ACTIVE  2
#define BIT_STATUS5_KNOCK_ACTIVE   3
#define BIT_STATUS5_KNOCK_PULSE    4
#define BIT_STATUS5_UNUSED6        5
#define BIT_STATUS5_UNUSED7        6
#define BIT_STATUS5_UNUSED8        7

#define BIT_TIMER_1HZ             0
#define BIT_TIMER_4HZ             1
#define BIT_TIMER_10HZ            2
#define BIT_TIMER_15HZ            3
#define BIT_TIMER_30HZ            4
#define BIT_TIMER_50HZ            5
#define BIT_TIMER_200HZ           6
#define BIT_TIMER_1KHZ            7

#define BIT_STATUS3_RESET_PREVENT 0 //Indicates whether reset prevention is enabled
#define BIT_STATUS3_NITROUS       1
#define BIT_STATUS3_FUEL2_ACTIVE  2
#define BIT_STATUS3_VSS_REFRESH   3
#define BIT_STATUS3_HALFSYNC      4 //shows if there is only sync from primary trigger, but not from secondary.
#define BIT_STATUS3_NSQUIRTS1     5
#define BIT_STATUS3_NSQUIRTS2     6
#define BIT_STATUS3_NSQUIRTS3     7

#define BIT_STATUS4_WMI_EMPTY     0 //Indicates whether the WMI tank is empty
#define BIT_STATUS4_VVT1_ERROR    1 //VVT1 cam angle within limits or not
#define BIT_STATUS4_VVT2_ERROR    2 //VVT2 cam angle within limits or not
#define BIT_STATUS4_FAN           3 //Fan Status
#define BIT_STATUS4_BURNPENDING   4
#define BIT_STATUS4_STAGING_ACTIVE 5
#define BIT_STATUS4_COMMS_COMPAT  6
#define BIT_STATUS4_ALLOW_LEGACY_COMMS       7

#define BIT_AIRCON_REQUEST        0 //Indicates whether the A/C button is pressed
#define BIT_AIRCON_COMPRESSOR     1 //Indicates whether the A/C compressor is running
#define BIT_AIRCON_RPM_LOCKOUT    2 //Indicates the A/C is locked out due to the RPM being too high/low, or the post-high/post-low-RPM "stand-down" lockout period
#define BIT_AIRCON_TPS_LOCKOUT    3 //Indicates the A/C is locked out due to high TPS, or the post-high-TPS "stand-down" lockout period
#define BIT_AIRCON_TURNING_ON     4 //Indicates the A/C request is on (i.e. A/C button pressed), the lockouts are off, however the start delay has not yet elapsed. This gives the idle up time to kick in before the compressor.
#define BIT_AIRCON_CLT_LOCKOUT    5 //Indicates the A/C is locked out either due to high coolant temp.
#define BIT_AIRCON_FAN            6 //Indicates whether the A/C fan is running
#define BIT_AIRCON_UNUSED8        7

#ifndef UNIT_TEST 
#define TOOTH_LOG_SIZE      127U
#else
#define TOOTH_LOG_SIZE      1U
#endif
// Some code relies on TOOTH_LOG_SIZE being uint8_t.
static_assert(TOOTH_LOG_SIZE<UINT8_MAX, "Check all uses of TOOTH_LOG_SIZE");

#define O2_CALIBRATION_PAGE   2U
#define IAT_CALIBRATION_PAGE  1U
#define CLT_CALIBRATION_PAGE  0U

// note the sequence of these defines which reference the bits used in a byte has moved when the third trigger & engine cycle was incorporated
#define COMPOSITE_LOG_PRI   0
#define COMPOSITE_LOG_SEC   1
#define COMPOSITE_LOG_THIRD 2 
#define COMPOSITE_LOG_TRIG 3
#define COMPOSITE_LOG_SYNC 4
#define COMPOSITE_ENGINE_CYCLE 5

#define EGO_TYPE_OFF      0
#define EGO_TYPE_NARROW   1
#define EGO_TYPE_WIDE     2

#define INJ_TYPE_PORT 0
#define INJ_TYPE_TBODY 1

#define INJ_PAIRED 0
#define INJ_SEMISEQUENTIAL 1
#define INJ_BANKED          2
#define INJ_SEQUENTIAL      3

#define INJ_PAIR_13_24      0
#define INJ_PAIR_14_23      1

#define OUTPUT_CONTROL_DIRECT   0
#define OUTPUT_CONTROL_MC33810  10

#define IGN_MODE_WASTED     0U
#define IGN_MODE_SINGLE     1U
#define IGN_MODE_WASTEDCOP  2U
#define IGN_MODE_SEQUENTIAL 3U
#define IGN_MODE_ROTARY     4U

#define SEC_TRIGGER_SINGLE  0
#define SEC_TRIGGER_4_1     1
#define SEC_TRIGGER_POLL    2
#define SEC_TRIGGER_5_3_2   3
#define SEC_TRIGGER_TOYOTA_3  4

#define ROTARY_IGN_FC       0
#define ROTARY_IGN_FD       1
#define ROTARY_IGN_RX8      2

#define BOOST_MODE_SIMPLE   0
#define BOOST_MODE_FULL     1

#define EN_BOOST_CONTROL_BARO   0
#define EN_BOOST_CONTROL_FIXED  1

#define WMI_MODE_SIMPLE       0
#define WMI_MODE_PROPORTIONAL 1
#define WMI_MODE_OPENLOOP     2
#define WMI_MODE_CLOSEDLOOP   3

#define HARD_CUT_FULL       0
#define HARD_CUT_ROLLING    1

#define EVEN_FIRE           0
#define ODD_FIRE            1

#define EGO_ALGORITHM_SIMPLE   0U
#define EGO_ALGORITHM_INVALID1 1U
#define EGO_ALGORITHM_PID      2U
#define EGO_ALGORITHM_NONE     3U

#define STAGING_MODE_TABLE  0
#define STAGING_MODE_AUTO   1

#define NITROUS_OFF         0
#define NITROUS_STAGE1      1
#define NITROUS_STAGE2      2
#define NITROUS_BOTH        3

#define PROTECT_CUT_OFF     0
#define PROTECT_CUT_IGN     1
#define PROTECT_CUT_FUEL    2
#define PROTECT_CUT_BOTH    3
#define PROTECT_IO_ERROR    7

#define AE_MODE_TPS         0
#define AE_MODE_MAP         1

#define AE_MODE_MULTIPLIER  0
#define AE_MODE_ADDER       1

#define KNOCK_MODE_OFF      0U
#define KNOCK_MODE_DIGITAL  1U
#define KNOCK_MODE_ANALOG   2U

#define KNOCK_TRIGGER_HIGH  0
#define KNOCK_TRIGGER_LOW   1

#define FUEL2_MODE_OFF      0
#define FUEL2_MODE_MULTIPLY 1
#define FUEL2_MODE_ADD      2
#define FUEL2_MODE_CONDITIONAL_SWITCH   3
#define FUEL2_MODE_INPUT_SWITCH 4

#define SPARK2_MODE_OFF      0
#define SPARK2_MODE_MULTIPLY 1
#define SPARK2_MODE_ADD      2
#define SPARK2_MODE_CONDITIONAL_SWITCH   3
#define SPARK2_MODE_INPUT_SWITCH 4

#define FUEL2_CONDITION_RPM 0
#define FUEL2_CONDITION_MAP 1
#define FUEL2_CONDITION_TPS 2
#define FUEL2_CONDITION_ETH 3

#define SPARK2_CONDITION_RPM 0
#define SPARK2_CONDITION_MAP 1
#define SPARK2_CONDITION_TPS 2
#define SPARK2_CONDITION_ETH 3

#define RESET_CONTROL_DISABLED             0U
#define RESET_CONTROL_PREVENT_WHEN_RUNNING 1U
#define RESET_CONTROL_PREVENT_ALWAYS       2U
#define RESET_CONTROL_SERIAL_COMMAND       3U

#define OPEN_LOOP_BOOST     0
#define CLOSED_LOOP_BOOST   1

#define SOFT_LIMIT_FIXED        0
#define SOFT_LIMIT_RELATIVE     1

#define VVT_MODE_ONOFF      0
#define VVT_MODE_OPEN_LOOP  1
#define VVT_MODE_CLOSED_LOOP 2
#define VVT_LOAD_MAP      0
#define VVT_LOAD_TPS      1

#define MULTIPLY_MAP_MODE_OFF   0
#define MULTIPLY_MAP_MODE_BARO  1
#define MULTIPLY_MAP_MODE_100   2

#define FOUR_STROKE         0U
#define TWO_STROKE          1U

#define GOING_LOW         0
#define GOING_HIGH        1

#define BATTV_COR_MODE_WHOLE 0
#define BATTV_COR_MODE_OPENTIME 1

#define INJ1_CMD_BIT      0
#define INJ2_CMD_BIT      1
#define INJ3_CMD_BIT      2
#define INJ4_CMD_BIT      3
#define INJ5_CMD_BIT      4
#define INJ6_CMD_BIT      5
#define INJ7_CMD_BIT      6
#define INJ8_CMD_BIT      7

#define IGN1_CMD_BIT      0
#define IGN2_CMD_BIT      1
#define IGN3_CMD_BIT      2
#define IGN4_CMD_BIT      3
#define IGN5_CMD_BIT      4
#define IGN6_CMD_BIT      5
#define IGN7_CMD_BIT      6
#define IGN8_CMD_BIT      7

#define ENGINE_PROTECT_BIT_RPM  0
#define ENGINE_PROTECT_BIT_MAP  1
#define ENGINE_PROTECT_BIT_OIL  2
#define ENGINE_PROTECT_BIT_AFR  3
#define ENGINE_PROTECT_BIT_COOLANT 4


#define CALIBRATION_TABLE_SIZE 512 ///< Calibration table size for CLT, IAT, O2
#define CALIBRATION_TEMPERATURE_OFFSET 40 /**< All temperature measurements are stored offset by 40 degrees.
This is so we can use an unsigned byte (0-255) to represent temperature ranges from -40 to 215 */
#define OFFSET_FUELTRIM 127U ///< The fuel trim tables are offset by 128 to allow for -128 to +128 values
#define OFFSET_IGNITION 40 ///< Ignition values from the main spark table are offset 40 degrees downwards to allow for negative spark timing

#define SERIAL_BUFFER_THRESHOLD 32 ///< When the serial buffer is filled to greater than this threshold value, the serial processing operations will be performed more urgently in order to avoid it overflowing. Serial buffer is 64 bytes long, so the threshold is set at half this as a reasonable figure

#define LOGGER_CSV_SEPARATOR_SEMICOLON  0
#define LOGGER_CSV_SEPARATOR_COMMA      1
#define LOGGER_CSV_SEPARATOR_TAB        2
#define LOGGER_CSV_SEPARATOR_SPACE      3

#define LOGGER_DISABLED                 0
#define LOGGER_CSV                      1
#define LOGGER_BINARY                   2

#define LOGGER_RATE_1HZ                 0
#define LOGGER_RATE_4HZ                 1
#define LOGGER_RATE_10HZ                2
#define LOGGER_RATE_30HZ                3

#define LOGGER_FILENAMING_OVERWRITE     0
#define LOGGER_FILENAMING_DATETIME      1
#define LOGGER_FILENAMING_SEQENTIAL     2

extern const char TSfirmwareVersion[] PROGMEM;

extern const byte data_structure_version; //This identifies the data structure when reading / writing. Now in use: CURRENT_DATA_VERSION (migration on-the fly) ?


//These are for the direct port manipulation of the injectors, coils and aux outputs
extern volatile PORT_TYPE *inj1_pin_port;
extern volatile PINMASK_TYPE inj1_pin_mask;
extern volatile PORT_TYPE *inj2_pin_port;
extern volatile PINMASK_TYPE inj2_pin_mask;
extern volatile PORT_TYPE *inj3_pin_port;
extern volatile PINMASK_TYPE inj3_pin_mask;
extern volatile PORT_TYPE *inj4_pin_port;
extern volatile PINMASK_TYPE inj4_pin_mask;
extern volatile PORT_TYPE *inj5_pin_port;
extern volatile PINMASK_TYPE inj5_pin_mask;
extern volatile PORT_TYPE *inj6_pin_port;
extern volatile PINMASK_TYPE inj6_pin_mask;
extern volatile PORT_TYPE *inj7_pin_port;
extern volatile PINMASK_TYPE inj7_pin_mask;
extern volatile PORT_TYPE *inj8_pin_port;
extern volatile PINMASK_TYPE inj8_pin_mask;

extern volatile PORT_TYPE *ign1_pin_port;
extern volatile PINMASK_TYPE ign1_pin_mask;
extern volatile PORT_TYPE *ign2_pin_port;
extern volatile PINMASK_TYPE ign2_pin_mask;
extern volatile PORT_TYPE *ign3_pin_port;
extern volatile PINMASK_TYPE ign3_pin_mask;
extern volatile PORT_TYPE *ign4_pin_port;
extern volatile PINMASK_TYPE ign4_pin_mask;
extern volatile PORT_TYPE *ign5_pin_port;
extern volatile PINMASK_TYPE ign5_pin_mask;
extern volatile PORT_TYPE *ign6_pin_port;
extern volatile PINMASK_TYPE ign6_pin_mask;
extern volatile PORT_TYPE *ign7_pin_port;
extern volatile PINMASK_TYPE ign7_pin_mask;
extern volatile PORT_TYPE *ign8_pin_port;
extern volatile PINMASK_TYPE ign8_pin_mask;

extern volatile PORT_TYPE *tach_pin_port;
extern volatile PINMASK_TYPE tach_pin_mask;
extern volatile PORT_TYPE *pump_pin_port;
extern volatile PINMASK_TYPE pump_pin_mask;

extern volatile PORT_TYPE *flex_pin_port;
extern volatile PINMASK_TYPE flex_pin_mask;

extern volatile PORT_TYPE *triggerPri_pin_port;
extern volatile PINMASK_TYPE triggerPri_pin_mask;
extern volatile PORT_TYPE *triggerSec_pin_port;
extern volatile PINMASK_TYPE triggerSec_pin_mask;
extern volatile PORT_TYPE *triggerThird_pin_port;
extern volatile PINMASK_TYPE triggerThird_pin_mask;

extern byte triggerInterrupt;
extern byte triggerInterrupt2;
extern byte triggerInterrupt3;


extern byte fpPrimeTime; //The time (in seconds, based on currentStatus.secl) that the fuel pump started priming
extern uint8_t softLimitTime; //The time (in 0.1 seconds, based on seclx10) that the soft limiter started
extern volatile uint16_t mainLoopCount;
extern uint32_t revolutionTime; //The time in uS that one revolution would take at current speed (The time tooth 1 was last seen, minus the time it was seen prior to that)
extern volatile unsigned long timer5_overflow_count; //Increments every time counter 5 overflows. Used for the fast version of micros()
extern volatile unsigned long ms_counter; //A counter that increments once per ms
extern uint16_t fixedCrankingOverride;
extern volatile uint32_t toothHistory[TOOTH_LOG_SIZE];
extern volatile uint8_t compositeLogHistory[TOOTH_LOG_SIZE];
extern volatile unsigned int toothHistoryIndex;
extern unsigned long currentLoopTime; /**< The time (in uS) that the current mainloop started */
extern volatile uint16_t ignitionCount; /**< The count of ignition events that have taken place since the engine started */
//The below shouldn't be needed and probably should be cleaned up, but the Atmel SAM (ARM) boards use a specific type for the trigger edge values rather than a simple byte/int
#if defined(CORE_SAMD21)
  extern PinStatus primaryTriggerEdge;
  extern PinStatus secondaryTriggerEdge;
  extern PinStatus tertiaryTriggerEdge;
#else
  extern byte primaryTriggerEdge;
  extern byte secondaryTriggerEdge;
  extern byte tertiaryTriggerEdge;
#endif
extern int CRANK_ANGLE_MAX_IGN;
extern int CRANK_ANGLE_MAX_INJ;       ///< The number of crank degrees that the system track over. 360 for wasted / timed batch and 720 for sequential
extern volatile uint32_t runSecsX10;  /**< Counter of seconds since cranking commenced (similar to runSecs) but in increments of 0.1 seconds */
extern volatile uint32_t seclx10;     /**< Counter of seconds since powered commenced (similar to secl) but in increments of 0.1 seconds */
extern volatile byte HWTest_INJ;      /**< Each bit in this variable represents one of the injector channels and it's HW test status */
extern volatile byte HWTest_INJ_Pulsed; /**< Each bit in this variable represents one of the injector channels and it's 50% HW test status */
extern volatile byte HWTest_IGN;      /**< Each bit in this variable represents one of the ignition channels and it's HW test status */
extern volatile byte HWTest_IGN_Pulsed; /**< Each bit in this variable represents one of the ignition channels and it's 50% HW test status */
extern byte maxIgnOutputs;            /**< Number of ignition outputs being used by the current tune configuration */
extern byte maxInjOutputs;            /**< Number of injection outputs being used by the current tune configuration */
extern byte resetControl; ///< resetControl needs to be here (as global) because using the config page (4) directly can prevent burning the setting
extern volatile byte TIMER_mask;
extern volatile byte LOOP_TIMER;


//These functions all do checks on a pin to determine if it is already in use by another (higher importance) function
#define pinIsInjector(pin)  ( ((pin) == pinInjector1) || ((pin) == pinInjector2) || ((pin) == pinInjector3) || ((pin) == pinInjector4) || ((pin) == pinInjector5) || ((pin) == pinInjector6) || ((pin) == pinInjector7) || ((pin) == pinInjector8) )
#define pinIsIgnition(pin)  ( ((pin) == pinCoil1) || ((pin) == pinCoil2) || ((pin) == pinCoil3) || ((pin) == pinCoil4) || ((pin) == pinCoil5) || ((pin) == pinCoil6) || ((pin) == pinCoil7) || ((pin) == pinCoil8) )
//#define pinIsOutput(pin)    ( pinIsInjector((pin)) || pinIsIgnition((pin)) || ((pin) == pinFuelPump) || ((pin) == pinFan) || ((pin) == pinAirConComp) || ((pin) == pinAirConFan)|| ((pin) == pinVVT_1) || ((pin) == pinVVT_2) || ( ((pin) == pinBoost) && configPage6.boostEnabled) || ((pin) == pinIdle1) || ((pin) == pinIdle2) || ((pin) == pinTachOut) || ((pin) == pinStepperEnable) || ((pin) == pinStepperStep) )
#define pinIsSensor(pin)    ( ((pin) == pinCLT) || ((pin) == pinIAT) || ((pin) == pinMAP) || ((pin) == pinTPS) || ((pin) == pinO2) || ((pin) == pinBat) || (((pin) == pinFlex) && (configPage2.flexEnabled != 0)) )
//#define pinIsUsed(pin)      ( pinIsSensor((pin)) || pinIsOutput((pin)) || pinIsReserved((pin)) )


/** The status struct with current values for all 'live' variables.
* In current version this is 64 bytes. Instantiated as global currentStatus.
* int *ADC (Analog-to-digital value / count) values contain the "raw" value from AD conversion, which get converted to
* unit based values in similar variable(s) without ADC part in name (see sensors.ino for reading of sensors).
*/
struct statuses {
  volatile bool hasSync : 1; /**< Flag for crank/cam position being known by decoders (See decoders.ino).
  This is used for sanity checking e.g. before logging tooth history or reading some sensors and computing readings. */
  bool initialisationComplete : 1; //Tracks whether the setup() function has run completely
  bool clutchTrigger : 1;
  bool previousClutchTrigger : 1;
  volatile bool fpPrimed : 1; //Tracks whether or not the fuel pump priming has been completed yet
  volatile bool injPrimed : 1; //Tracks whether or not the injector priming has been completed yet
  volatile bool tachoSweepEnabled : 1;
  volatile bool tachoAlt : 1;
    
  uint16_t RPM;   ///< RPM - Current Revs per minute
  byte RPMdiv100; ///< RPM value scaled (divided by 100) to fit a byte (0-255, e.g. 12000 => 120)
  long longRPM;   ///< RPM as long int (gets assigned to / maintained in statuses.RPM as well)
  uint16_t baroADC;
  long MAP;     ///< Manifold absolute pressure. Has to be a long for PID calcs (Boost control)
  int16_t EMAP; ///< EMAP ... (See @ref config6.useEMAP for EMAP enablement)
  uint8_t baro;   ///< Barometric pressure is simply the initial MAP reading, taken before the engine is running. Alternatively, can be taken from an external sensor
  uint8_t TPS;    /**< The current TPS reading (0% - 100%). Is the tpsADC value after the calibration is applied */
  uint8_t tpsADC; /**< byte (valued: 0-255) representation of the TPS. Downsampled from the original 10-bit (0-1023) reading, but before any calibration is applied */
  int16_t tpsDOT; /**< TPS delta over time. Measures the % per second that the TPS is changing. Note that is signed value, because TPSdot can be also negative */
  byte TPSlast; /**< The previous TPS reading */
  int16_t mapDOT; /**< MAP delta over time. Measures the kpa per second that the MAP is changing. Note that is signed value, because MAPdot can be also negative */
  volatile int rpmDOT; /**< RPM delta over time (RPM increase / s ?) */
  byte VE;     /**< The current VE value being used in the fuel calculation. Can be the same as VE1 or VE2, or a calculated value of both. */
  byte VE1;    /**< The VE value from fuel table 1 */
  byte VE2;    /**< The VE value from fuel table 2, if in use (and required conditions are met) */
  uint8_t O2;     /**< Primary O2 sensor reading */
  uint8_t O2_2;   /**< Secondary O2 sensor reading */
  int coolant; /**< Coolant temperature reading */
  uint16_t cltADC;
  int IAT;     /**< Inlet air temperature reading */
  uint16_t iatADC;
  uint16_t O2ADC;
  uint16_t O2_2ADC;
  uint16_t dwell;          ///< dwell (coil primary winding/circuit on) time (in ms * 10 ? See @ref correctionsDwell)
  volatile uint16_t actualDwell;    ///< actual dwell time if new ignition mode is used (in uS)
  byte dwellCorrection; /**< The amount of correction being applied to the dwell time (in unit ...). */
  byte battery10;     /**< The current BRV in volts (multiplied by 10. Eg 12.5V = 125) */
  int8_t advance;     /**< The current advance value being used in the spark calculation. Can be the same as advance1 or advance2, or a calculated value of both */
  int8_t advance1;    /**< The advance value from ignition table 1 */
  int8_t advance2;    /**< The advance value from ignition table 2 */
  uint16_t corrections; /**< The total current corrections % amount */
  uint16_t AEamount;    /**< The amount of acceleration enrichment currently being applied. 100=No change. Varies above 255 */
  byte egoCorrection; /**< The amount of closed loop AFR enrichment currently being applied */
  byte wueCorrection; /**< The amount of warmup enrichment currently being applied */
  byte batCorrection; /**< The amount of battery voltage enrichment currently being applied */
  byte iatCorrection; /**< The amount of inlet air temperature adjustment currently being applied */
  byte baroCorrection; /**< The amount of correction being applied for the current baro reading */
  byte launchCorrection;   /**< The amount of correction being applied if launch control is active */
  byte flexCorrection;     /**< Amount of correction being applied to compensate for ethanol content */
  byte fuelTempCorrection; /**< Amount of correction being applied to compensate for fuel temperature */
  int8_t flexIgnCorrection;/**< Amount of additional advance being applied based on flex. Note the type as this allows for negative values */
  byte afrTarget;    /**< Current AFR Target looked up from AFR target table (x10 ? See @ref afrTable)*/
  byte CLIdleTarget; /**< The target idle RPM (when closed loop idle control is active) */
  bool idleUpActive; /**< Whether the externally controlled idle up is currently active */
  bool CTPSActive;   /**< Whether the externally controlled closed throttle position sensor is currently active */
  volatile byte ethanolPct; /**< Ethanol reading (if enabled). 0 = No ethanol, 100 = pure ethanol. Eg E85 = 85. */
  volatile int8_t fuelTemp;
  unsigned long AEEndTime; /**< The target end time used whenever AE (acceleration enrichment) is turned on */
  volatile byte status1; ///< Status bits (See BIT_STATUS1_* defines on top of this file)
  volatile byte status2;   ///< status 2/control indicator bits (launch control, boost cut, spark errors, See BIT_STATUS2_* defines)
  volatile byte status3; ///< Status bits (See BIT_STATUS3_* defines on top of this file)
  volatile byte status4; ///< Status bits (See BIT_STATUS4_* defines on top of this file)
  volatile byte status5;  ///< Status 5 ... (See also @ref config10 Status 5* members and BIT_STATU5_* defines)
  uint8_t engine; ///< Engine status bits (See BIT_ENGINE_* defines on top of this file)
  unsigned int PW1; ///< In uS
  unsigned int PW2; ///< In uS
  unsigned int PW3; ///< In uS
  unsigned int PW4; ///< In uS
  unsigned int PW5; ///< In uS
  unsigned int PW6; ///< In uS
  unsigned int PW7; ///< In uS
  unsigned int PW8; ///< In uS
  volatile byte runSecs; /**< Counter of seconds since cranking commenced (Maxes out at 255 to prevent overflow) */
  volatile byte secl; /**< Counter incrementing once per second. Will overflow after 255 and begin again. This is used by TunerStudio to maintain comms sync */
  volatile uint16_t loopsPerSecond; /**< A performance indicator showing the number of main loops that are being executed each second */ 
  bool launchingSoft; /**< Indicator showing whether soft launch control adjustments are active */
  bool launchingHard; /**< Indicator showing whether hard launch control adjustments are active */
  uint16_t freeRAM;
  unsigned int clutchEngagedRPM; /**< The RPM at which the clutch was last depressed. Used for distinguishing between launch control and flat shift */ 
  bool flatShiftingHard;
  volatile uint32_t startRevolutions; /**< A counter for how many revolutions have been completed since sync was achieved. */
  uint16_t boostTarget;
  byte testOutputs;   ///< Test Output bits (only first bit used/tested ?)
  bool testActive;    // Not in use ? Replaced by testOutputs ?
  uint16_t boostDuty; ///< Boost Duty percentage value * 100 to give 2 points of precision
  byte idleLoad;      ///< Either the current steps or current duty cycle for the idle control
  uint16_t canin[16]; ///< 16bit raw value of selected canin data for channels 0-15
  uint8_t current_caninchannel = 0; /**< Current CAN channel, defaults to 0 */
  uint16_t crankRPM = 400; /**< The actual cranking RPM limit. This is derived from the value in the config page, but saves us multiplying it every time it's used (Config page value is stored divided by 10) */
  int16_t flexBoostCorrection; /**< Amount of boost added based on flex */
  byte nitrous_status;
  byte nSquirts;  ///< Number of injector squirts per cycle (per injector)
  byte nChannels; /**< Number of fuel and ignition channels.  */
  int16_t fuelLoad;
  int16_t fuelLoad2;
  int16_t ignLoad;
  int16_t ignLoad2;
  bool fuelPumpOn; /**< Indicator showing the current status of the fuel pump */
  volatile byte syncLossCounter;
  byte knockRetard;
  volatile byte knockCount;
  bool toothLogEnabled;
  byte compositeTriggerUsed; // 0 means composite logger disabled, 2 means use secondary input (1st cam), 3 means use tertiary input (2nd cam), 4 means log both cams together
  int16_t vvt1Angle; //Has to be a long for PID calcs (CL VVT control)
  byte vvt1TargetAngle;
  long vvt1Duty; //Has to be a long for PID calcs (CL VVT control)
  uint16_t injAngle;
  byte ASEValue;
  uint16_t vss;      /**< Current speed reading. Natively stored in kph and converted to mph in TS if required */
  bool idleUpOutputActive; /**< Whether the idle up output is currently active */
  byte gear;         /**< Current gear (Calculated from vss) */
  byte fuelPressure; /**< Fuel pressure in PSI */
  byte oilPressure;  /**< Oil pressure in PSI */
  byte engineProtectStatus;
  byte fanDuty;
  byte wmiPW;
  int16_t vvt2Angle; //Has to be a long for PID calcs (CL VVT control)
  byte vvt2TargetAngle;
  long vvt2Duty; //Has to be a long for PID calcs (CL VVT control)
  byte outputsStatus;
  byte TS_SD_Status; //TunerStudios SD card status
  byte airConStatus;
};

/**
 * @brief Non-atomic version of HasAnySync. **Should only be called in an ATOMIC() block***
 * 
 */
static inline bool HasAnySyncUnsafe(const statuses &status) {
  return status.hasSync || BIT_CHECK(status.status3, BIT_STATUS3_HALFSYNC);
}

static inline bool HasAnySync(const statuses &status) {
  ATOMIC() {
    return HasAnySyncUnsafe(status);
  }
  return false; // Just here to avoid compiler warning.
}


#define IDLEADVANCE_MODE_OFF      0U
#define IDLEADVANCE_MODE_ADDED    1U
#define IDLEADVANCE_MODE_SWITCHED 2U

#define IDLEADVANCE_ALGO_TPS      0U
#define IDLEADVANCE_ALGO_CTPS     1U


extern byte pinInjector1; //Output pin injector 1
extern byte pinInjector2; //Output pin injector 2
extern byte pinInjector3; //Output pin injector 3
extern byte pinInjector4; //Output pin injector 4
extern byte pinInjector5; //Output pin injector 5
extern byte pinInjector6; //Output pin injector 6
extern byte pinInjector7; //Output pin injector 7
extern byte pinInjector8; //Output pin injector 8
extern byte injectorOutputControl; //Specifies whether the injectors are controlled directly (Via an IO pin) or using something like the MC33810
extern byte pinCoil1; //Pin for coil 1
extern byte pinCoil2; //Pin for coil 2
extern byte pinCoil3; //Pin for coil 3
extern byte pinCoil4; //Pin for coil 4
extern byte pinCoil5; //Pin for coil 5
extern byte pinCoil6; //Pin for coil 6
extern byte pinCoil7; //Pin for coil 7
extern byte pinCoil8; //Pin for coil 8
extern byte ignitionOutputControl; //Specifies whether the coils are controlled directly (Via an IO pin) or using something like the MC33810
extern byte pinTrigger; //The CAS pin
extern byte pinTrigger2; //The Cam Sensor pin known as secondary input
extern byte pinTrigger3;	//the 2nd cam sensor pin known as tertiary input
extern byte pinTPS;//TPS input pin
extern byte pinMAP; //MAP sensor pin
extern byte pinEMAP; //EMAP sensor pin
extern byte pinMAP2; //2nd MAP sensor (Currently unused)
extern byte pinIAT; //IAT sensor pin
extern byte pinCLT; //CLS sensor pin
extern byte pinO2; //O2 Sensor pin
extern byte pinO2_2; //second O2 pin
extern byte pinBat; //Battery voltage pin
extern byte pinDisplayReset; // OLED reset pin
extern byte pinTachOut; //Tacho output
extern byte pinFuelPump; //Fuel pump on/off
extern byte pinIdle1; //Single wire idle control
extern byte pinIdle2; //2 wire idle control (Not currently used)
extern byte pinIdleUp; //Input for triggering Idle Up
extern byte pinIdleUpOutput; //Output that follows (normal or inverted) the idle up pin
extern byte pinCTPS; //Input for triggering closed throttle state
extern byte pinFuel2Input; //Input for switching to the 2nd fuel table
extern byte pinSpark2Input; //Input for switching to the 2nd ignition table
extern byte pinSpareTemp1; // Future use only
extern byte pinSpareTemp2; // Future use only
extern byte pinSpareOut1; //Generic output
extern byte pinSpareOut2; //Generic output
extern byte pinSpareOut3; //Generic output
extern byte pinSpareOut4; //Generic output
extern byte pinSpareOut5; //Generic output
extern byte pinSpareOut6; //Generic output
extern byte pinSpareHOut1; //spare high current output
extern byte pinSpareHOut2; // spare high current output
extern byte pinSpareLOut1; // spare low current output
extern byte pinSpareLOut2; // spare low current output
extern byte pinSpareLOut3;
extern byte pinSpareLOut4;
extern byte pinSpareLOut5;
extern byte pinBoost;
extern byte pinVVT_1;		// vvt output 1
extern byte pinVVT_2;		// vvt output 2
extern byte pinFan;       // Cooling fan output
extern byte pinStepperDir; //Direction pin for the stepper motor driver
extern byte pinStepperStep; //Step pin for the stepper motor driver
extern byte pinStepperEnable; //Turning the DRV8825 driver on/off
extern byte pinLaunch;
extern byte pinIgnBypass; //The pin used for an ignition bypass (Optional)
extern byte pinFlex; //Pin with the flex sensor attached
extern byte pinVSS; 
extern byte pinBaro; //Pin that an external barometric pressure sensor is attached to (If used)
extern byte pinResetControl; // Output pin used control resetting the Arduino
extern byte pinFuelPressure;
extern byte pinOilPressure;
extern byte pinWMIEmpty; // Water tank empty sensor
extern byte pinWMIIndicator; // No water indicator bulb
extern byte pinWMIEnabled; // ON-OFF output to relay/pump/solenoid
extern byte pinMC33810_1_CS;
extern byte pinMC33810_2_CS;
extern byte pinSDEnable; //Input for manually enabling SD logging
#ifdef USE_SPI_EEPROM
  extern byte pinSPIFlash_CS;
#endif
extern byte pinAirConComp;    // Air conditioning compressor output
extern byte pinAirConFan;    // Stand-alone air conditioning fan output
extern byte pinAirConRequest; // Air conditioning request input

extern struct statuses currentStatus; //The global status object
/* global variables */ // from speeduino.ino
//#ifndef UNIT_TEST

//#endif


#endif // GLOBALS_H
