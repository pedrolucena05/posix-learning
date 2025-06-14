#ifndef HANDLESIG_H
#define HANDLESIG_H

#include <signal.h>  
#include <stdio.h>   

#include "dStruct.h"

extern carData *memory; 

void handleSignal(int sig);
void handlePause(int sig);

#endif 
