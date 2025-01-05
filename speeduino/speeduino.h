/** \file speeduino.h
 * @brief Speeduino main file containing initial setup and system loop functions 
 * @author Josh Stewart
 * 
 * This file contains the main system loop of the Speeduino core and thus much of the logic of the fuel and ignition algorithms is contained within this
 * It is where calls to all the auxiliary control systems, sensor reads, comms etc are made
 * 
 * It also contains the setup() function that is called by the bootloader on system startup
 * 
 */

#ifndef SPEEDUINO_H
#define SPEEDUINO_H
//#include "globals.h"

void setup(void);
void loop(void);


bool pinIsOutput(byte pin);
bool pinIsUsed(byte pin);

extern byte loopTimerMask;
extern uint32_t currentLoopTime; /**< The time (in uS) that the current mainloop started */



#endif
