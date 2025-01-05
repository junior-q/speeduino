#ifndef DECODER_MISSING_TOOTH_H
#define DECODER_MISSING_TOOTH_H


//All of the below are the 6 required functions for each decoder / pattern
void triggerSetup_missingTooth(void);
void triggerPri_missingTooth(void);
void triggerSec_missingTooth(void);
void triggerThird_missingTooth(void);
uint16_t getRPM_missingTooth(void);
int getCrankAngle_missingTooth(void);
extern void triggerSetEndTeeth_missingTooth(void);


#endif
