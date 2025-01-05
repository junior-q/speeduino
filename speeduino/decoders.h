#ifndef DECODERS_H
#define DECODERS_H

#include "globals.h"

#include "decoders/decoder_24X.h"
#include "decoders/decoder_missingTooth.h"
#include "decoders/decoder_420a.h"
#include "decoders/decoder_4G63.h"
#include "decoders/decoder_Audi135.h"
#include "decoders/decoder_BasicDistributor.h"
#include "decoders/decoder_Daihatsu.h"
#include "decoders/decoder_DRZ400.h"
#include "decoders/decoder_DualWheel.h"
#include "decoders/decoder_FordST170.h"
#include "decoders/decoder_GM7X.h"
#include "decoders/decoder_Harley.h"
#include "decoders/decoder_HondaD17.h"
#include "decoders/decoder_HondaJ32.h"
#include "decoders/decoder_Jeep2000.h"
#include "decoders/decoder_MazdaAU.h"
#include "decoders/decoder_Miata9905.h"
#include "decoders/decoder_missingTooth.h"
#include "decoders/decoder_NGC.h"
#include "decoders/decoder_Nissan360.h"
#include "decoders/decoder_non360.h"
#include "decoders/decoder_Renix.h"
#include "decoders/decoder_RoverMEMS.h"
#include "decoders/decoder_Subaru67.h"
#include "decoders/decoder_SuzukiK6A.h"
#include "decoders/decoder_ThirtySixMinus21.h"
#include "decoders/decoder_ThirtySixMinus222.h"
#include "decoders/decoder_Vmax.h"
#include "decoders/decoder_Webber.h"



#if defined(CORE_AVR)
  #define READ_PRI_TRIGGER() ((*triggerPri_pin_port & triggerPri_pin_mask) ? true : false)
  #define READ_SEC_TRIGGER() ((*triggerSec_pin_port & triggerSec_pin_mask) ? true : false)
  #define READ_THIRD_TRIGGER() ((*triggerThird_pin_port & triggerThird_pin_mask) ? true : false)
#else
  #define READ_PRI_TRIGGER() digitalRead(pinTrigger)
  #define READ_SEC_TRIGGER() digitalRead(pinTrigger2)
  #define READ_THIRD_TRIGGER() digitalRead(pinTrigger3)  
#endif

#define DECODER_MISSING_TOOTH     0
#define DECODER_BASIC_DISTRIBUTOR 1
#define DECODER_DUAL_WHEEL        2
#define DECODER_GM7X              3
#define DECODER_4G63              4
#define DECODER_24X               5
#define DECODER_JEEP2000          6
#define DECODER_AUDI135           7
#define DECODER_HONDA_D17         8
#define DECODER_MIATA_9905        9
#define DECODER_MAZDA_AU          10
#define DECODER_NON360            11
#define DECODER_NISSAN_360        12
#define DECODER_SUBARU_67         13
#define DECODER_DAIHATSU_PLUS1    14
#define DECODER_HARLEY            15
#define DECODER_36_2_2_2          16
#define DECODER_36_2_1            17
#define DECODER_420A              18
#define DECODER_WEBER             19
#define DECODER_ST170             20
#define DECODER_DRZ400            21
#define DECODER_NGC               22
#define DECODER_VMAX              23
#define DECODER_RENIX             24
#define DECODER_ROVERMEMS		  25
#define DECODER_SUZUKI_K6A        26
#define DECODER_HONDA_J32         27

#define BIT_DECODER_2ND_DERIV           0 //The use of the 2nd derivative calculation is limited to certain decoders. This is set to either true or false in each decoders setup routine
#define BIT_DECODER_IS_SEQUENTIAL       1 //Whether or not the decoder supports sequential operation
#define BIT_DECODER_UNUSED1             2 
#define BIT_DECODER_HAS_SECONDARY       3 //Whether or not the decoder supports fixed cranking timing
#define BIT_DECODER_HAS_FIXED_CRANKING  4
#define BIT_DECODER_VALID_TRIGGER       5 //Is set true when the last trigger (Primary or secondary) was valid (ie passed filters)
#define BIT_DECODER_TOOTH_ANG_CORRECT   6 //Whether or not the triggerInfo.triggerToothAngle variable is currently accurate. Some patterns have times when the triggerInfo.triggerToothAngle variable cannot be accurately set.

#define TRIGGER_FILTER_OFF              0
#define TRIGGER_FILTER_LITE             1
#define TRIGGER_FILTER_MEDIUM           2
#define TRIGGER_FILTER_AGGRESSIVE       3


#define CRANK_SPEED 0U
#define CAM_SPEED   1U

#define TOOTH_CRANK 0
#define TOOTH_CAM_SECONDARY 1
#define TOOTH_CAM_TERTIARY  2

// used by the ROVER MEMS pattern
#define ID_TOOTH_PATTERN 0 // have we identified teeth to skip for calculating RPM?
#define SKIP_TOOTH1 1
#define SKIP_TOOTH2 2
#define SKIP_TOOTH3 3
#define SKIP_TOOTH4 4


/**
 * @brief Is the engine running?
 * 
 * This is based on whether or not the decoder has detected a tooth recently
 * 
 * @param triggerInfo.curTime The time in µS to use for the liveness check. Typically the result of a recent call to micros() 
 * @return true If the engine is turning
 * @return false If the engine is not turning
 */
bool engineIsRunning(uint32_t curTime);

/*
extern volatile bool validTrigger; //Is set true when the last trigger (Primary or secondary) was valid (ie passed filters)
extern volatile bool triggerInfo.triggerToothAngleIsCorrect; //Whether or not the triggerInfo.triggerToothAngle variable is currently accurate. Some patterns have times when the triggerInfo.triggerToothAngle variable cannot be accurately set.
extern bool secondDerivEnabled; //The use of the 2nd derivative calculation is limited to certain decoders. This is set to either true or false in each decoders setup routine
extern bool decoderIsSequential; //Whether or not the decoder supports sequential operation
extern bool decoderHasSecondary; //Whether or not the pattern uses a secondary input
extern bool decoderHasFixedCrankingTiming; 
*/

void loggerPrimaryISR(void);
void loggerSecondaryISR(void);
void loggerTertiaryISR(void);


/**
 * @brief This function is called when the engine is stopped, or when the engine is started. It resets the decoder state and the tooth tracking variables
 *
 * @return void
 */
void resetDecoder(void);

extern void (*triggerHandler)(void); //Pointer for the trigger function (Gets pointed to the relevant decoder)
extern void (*triggerSecondaryHandler)(void); //Pointer for the secondary trigger function (Gets pointed to the relevant decoder)
extern void (*triggerTertiaryHandler)(void); //Pointer for the tertiary trigger function (Gets pointed to the relevant decoder)

extern uint16_t (*getRPM)(void); //Pointer to the getRPM function (Gets pointed to the relevant decoder)
extern int (*getCrankAngle)(void); //Pointer to the getCrank Angle function (Gets pointed to the relevant decoder)
extern void (*triggerSetEndTeeth)(void); //Pointer to the triggerSetEndTeeth function of each decoder

extern uint32_t MAX_STALL_TIME; 			//The maximum time (in uS) that the system will continue to function before the engine is considered stalled/stopped. This is unique to each decoder, depending on the number of teeth etc. 500000 (half a second) is used as the default value, most decoders will be much less.


struct triggerInfo_t{
	volatile uint8_t decoderState;

    volatile uint32_t curTime;
    volatile uint32_t curGap;
    volatile uint32_t curTime2;
    volatile uint32_t curGap2;
    volatile uint32_t curTime3;
    volatile uint32_t curGap3;
    volatile uint32_t lastGap;
    volatile uint32_t targetGap;

    volatile uint16_t toothCurrentCount; 					//The current number of teeth (Once sync has been achieved, this can never actually be 0
    volatile uint32_t toothSystemLastToothTime; 			//As below, but used for decoders where not every tooth count is used for calculation
    volatile uint32_t toothLastToothTime; 					//The time (micros()) that the last tooth was registered
    volatile uint16_t secondaryToothCount; 			//Used for identifying the current secondary (Usually cam) tooth for patterns with multiple secondary teeth

    volatile uint32_t toothLastSecToothTime; 				//The time (micros()) that the last tooth was registered on the secondary input
    volatile uint32_t toothLastThirdToothTime; 				//The time (micros()) that the last tooth was registered on the second cam input

    volatile uint32_t toothLastMinusOneToothTime; 			//The time (micros()) that the tooth before the last tooth was registered
    volatile uint32_t toothLastMinusOneSecToothTime; 		//The time (micros()) that the tooth before the last tooth was registered on secondary input
    volatile uint32_t toothLastToothRisingTime; 			//The time (micros()) that the last tooth rose (used by special decoders to determine missing teeth polarity)
    volatile uint32_t toothLastSecToothRisingTime; 			//The time (micros()) that the last tooth rose on the secondary input (used by special decoders to determine missing teeth polarity)
    volatile uint32_t targetGap2;
    volatile uint32_t targetGap3;
    volatile uint32_t toothOneTime; 						//The time (micros()) that tooth 1 last triggered
    volatile uint32_t toothOneMinusOneTime; 				//The 2nd to last time (micros()) that tooth 1 last triggered
    volatile bool 	  revolutionOne; 						// For sequential operation, this tracks whether the current revolution is 1 or 2 (not 1)
    volatile bool 	  revolutionLastOne; 					// used to identify in the rover pattern which has a non unique primary trigger something unique - has the secondary tooth changed.
    volatile byte 	  toothSystemCount; 					//Used for decoders such as Audi 135 where not every tooth is used for calculating crank angle. This variable stores the actual number of teeth, not the number being used to calculate crank angle

    volatile uint16_t secondaryLastToothCount; 				// used to identify in the rover pattern which has a non unique primary trigger something unique - has the secondary tooth changed.
    volatile uint32_t secondaryLastToothTime; 				//The time (micros()) that the last tooth was registered (Cam input)
    volatile uint32_t secondaryLastToothTime1; 				//The time (micros()) that the last tooth was registered (Cam input)

    volatile uint16_t thirdToothCount; 					//Used for identifying the current third (Usually exhaust cam - used for VVT2) tooth for patterns with multiple secondary teeth
    volatile uint32_t thirdLastToothTime; 					//The time (micros()) that the last tooth was registered (Cam input)
    volatile uint32_t thirdLastToothTime1; 					//The time (micros()) that the last tooth was registered (Cam input)

    uint16_t          triggerActualTeeth;
    volatile uint32_t triggerFilterTime; // The shortest time (in uS) that pulses will be accepted (Used for debounce filtering)
    volatile uint32_t triggerSecFilterTime; // The shortest time (in uS) that pulses will be accepted (Used for debounce filtering) for the secondary input
    volatile uint32_t triggerThirdFilterTime; // The shortest time (in uS) that pulses will be accepted (Used for debounce filtering) for the Third input

    uint16_t triggerSecFilterTime_duration; // The shortest valid time (in uS) pulse DURATION
    volatile uint16_t triggerToothAngle; //The number of crank degrees that elapse per tooth
    uint32_t elapsedTime;
    uint32_t lastCrankAngleCalc;
    uint32_t lastVVTtime; //The time between the vvt reference pulse and the last crank pulse
    byte checkSyncToothCount; //How many teeth must've been seen on this revolution before we try to confirm sync (Useful for missing tooth type decoders)

    uint16_t ignition1EndTooth;
    uint16_t ignition2EndTooth;
    uint16_t ignition3EndTooth;
    uint16_t ignition4EndTooth;
    uint16_t ignition5EndTooth;
    uint16_t ignition6EndTooth;
    uint16_t ignition7EndTooth;
    uint16_t ignition8EndTooth;

    int16_t toothAngles[24]; //An array for storing fixed tooth angles. Currently sized at 24 for the GM 24X decoder, but may grow later if there are other decoders that use this style

};

extern triggerInfo_t	triggerInfo;


#endif
