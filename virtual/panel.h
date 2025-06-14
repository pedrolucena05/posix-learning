#ifndef PANEL_H
#define PANEL_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <semaphore.h>
#include <mqueue.h>
#include <string.h>
#include "handleSig.h"
#include "dStruct.h"  

extern sem_t *semB;    
extern sem_t *semC;    
extern carData *memory;

void panelProcess();

#endif 
