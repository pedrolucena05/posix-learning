#include "sensor.h"

pthread_t threadVelocity, threadRotation, threadTemperature;

void *generateVelocity(void *arg) {
    printf("(Sensor) Gerando Velocidade...\n");

    if (memory->exitFlag != 0) {
        return NULL;
    }

    memory->carVelocity = rand() % 201;

    printf("(Sensor) Velocidade gerada: %d\n", memory->carVelocity);

    return NULL;
}

void *generateRotation(void *arg) {
    pthread_t *threadToWaitFor = (pthread_t *)arg; 

    // Aguarda a thread anterior (Velocity) terminar
    pthread_join(*threadToWaitFor, NULL);

    if (memory->exitFlag != 0) {
        return NULL;
    }

    printf("(Sensor) Gerando Rotação...\n");

    memory->enRotation = memory->carVelocity * 37;
    printf("(Sensor) Rotação gerada: %d\n", memory->enRotation);

    return NULL;
}

void *generateTemperature(void *arg) {
    pthread_t *threadToWaitFor = (pthread_t *)arg; 

    // Aguarda a thread anterior (Rotation) terminar
    pthread_join(*threadToWaitFor, NULL);

    if (memory->exitFlag != 0) {
        return NULL;
    }

    printf("(Sensor) Gerando Temperatura...\n");

    memory->enTemp = (rand() % 91) + 55;
    printf("(Sensor) Temperatura gerada: %d\n", memory->enTemp);

    return NULL;
}

void sensorProcess() {
    while (memory->exitFlag == 0) {
        sem_wait(semA);

        if (memory->exitFlag != 0) {
            sem_post(semB);
            break;
        }

        printf("(Sensor) Processo Sensor executando...\n");

        // Cria as threads na ordem de dependência
        pthread_create(&threadVelocity, NULL, generateVelocity, NULL);
        pthread_create(&threadRotation, NULL, generateRotation, &threadVelocity);
        pthread_create(&threadTemperature, NULL, generateTemperature, &threadRotation);

        // Aguarda a última thread (Temperature) terminar
        pthread_join(threadTemperature, NULL);

        printf("(Sensor) Todas as threads concluíram a execução.\n");

        sem_post(semB);
    }
}

