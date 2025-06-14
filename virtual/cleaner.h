#ifndef CLEANER_H
#define CLEANER_H

#include <semaphore.h>
#include <sys/mman.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>

#include "dStruct.h"

extern sem_t *semA, *semB, *semC;
extern carData *memory;
extern int msID;

void cleanup();

#endif 
