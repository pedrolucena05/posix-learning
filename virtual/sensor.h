#ifndef SENSOR_H
#define SENSOR_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <semaphore.h>
#include <pthread.h>
#include "dStruct.h" 

extern sem_t *semA;    
extern sem_t *semB;    
extern carData *memory; 

void sensorProcess();

#endif 

