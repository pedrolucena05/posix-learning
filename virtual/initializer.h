#ifndef INITIALIZER_H
#define INITIALIZER_H

#include <signal.h>   
#include <stdio.h>    
#include <stdlib.h>   
#include <unistd.h>   
#include <semaphore.h>
#include <sys/mman.h> 
#include <fcntl.h>  
#include <mqueue.h>  

#include "handleSig.h"
#include "dStruct.h"  

extern sem_t *semA, *semB, *semC;
extern carData *memory;
extern int msID;

void cleanup();  

void generalInit();

#endif
