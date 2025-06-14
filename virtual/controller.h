#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <semaphore.h>
#include <mqueue.h>
#include "dStruct.h"  
#include "handleSig.h"

extern sem_t *semA;    
extern sem_t *semC;    
extern carData *memory;

void controllerProcess();

#endif 

