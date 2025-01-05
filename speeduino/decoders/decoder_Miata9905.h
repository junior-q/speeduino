#ifndef DECODER_MIATA_9905_H
#define DECODER_MIATA_9905_H


void triggerSetup_Miata9905(void);
void triggerPri_Miata9905(void);
void triggerSec_Miata9905(void);
uint16_t getRPM_Miata9905(void);
int getCrankAngle_Miata9905(void);
void triggerSetEndTeeth_Miata9905(void);
int getCamAngle_Miata9905(void);


#endif
