#include "cleaner.h"

void cleanup() {
    
    sem_close(semA);
    sem_close(semB);
    sem_close(semC);

    // Desalocar dos semaforos
    sem_unlink("/semA");
    sem_unlink("/semB");
    sem_unlink("/semC");

    // Desmapeamento e fechamento da memória compartilhada
    munmap(memory, sizeof(carData));  // Desmapeia toda a memória compartilhada
    close(msID);
    shm_unlink("/memorySensor");

    printf("\nRecursos limpos com sucesso.\n");
}
