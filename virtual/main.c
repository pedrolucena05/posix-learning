#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <semaphore.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <sys/wait.h>
#include <mqueue.h>
#include <pthread.h>

#include "handleSig.h"
#include "dStruct.h"
#include "initializer.h"
#include "cleaner.h"
#include "sensor.h"
#include "panel.h"
#include "controller.h"

sem_t *semA, *semB, *semC;
carData* memory;
int msID;
pid_t pSensors, pPanel;

int main() {

    generalInit();
    printf("\n(main) Processo Controlador iniciado.\n\n");
    
    pSensors = fork();
    if (pSensors == 0) {
        // Ignora SIGUSR2 no filho
        signal(SIGUSR2, SIG_IGN);
        signal(SIGUSR1, SIG_IGN);
        sensorProcess();
        exit(0);
    }

    pPanel = fork();
    if (pPanel == 0) {
        // Ignora SIGUSR2 no filho
        signal(SIGUSR2, SIG_IGN);
        signal(SIGUSR1, SIG_IGN);
        panelProcess();
        exit(0);
    }
    
    //Entra no loop do controlador
    controllerProcess();
    
    waitpid(pSensors, NULL, 0);  
    waitpid(pPanel, NULL, 0); 

    printf("\n(Relatorio final) Quantidade de vezes que o ADAS atuou: %d", memory->adasQTD);
    printf("\n(Relatorio final) Quantidade de vezes que os Atuadores foram usados: %d", memory->atQTD);
    
    cleanup(); 

    return 0;
}

