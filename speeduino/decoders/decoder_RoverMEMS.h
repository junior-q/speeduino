#ifndef DECODER_ROVER_MEMS_H
#define DECODER_ROVER_MEMS_H

void triggerSetup_RoverMEMS(void);
void triggerPri_RoverMEMS(void);
void triggerSec_RoverMEMS(void);
uint16_t getRPM_RoverMEMS(void);
int getCrankAngle_RoverMEMS(void);
void triggerSetEndTeeth_RoverMEMS(void);


#endif
