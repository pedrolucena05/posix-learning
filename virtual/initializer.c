#include "initializer.h"

void generalInit () {
    // Configura o manipulador de sinal no processo principal
    signal(SIGUSR2, handleSignal);
    signal(SIGUSR1, handlePause);

    // Configuração da memória compartilhada
    shm_unlink("/memorySensor");
    msID = shm_open("/memorySensor", O_CREAT | O_RDWR, 0666);
    if (msID == -1) {
        perror("shm_open falhou");
        exit(1);
    }

    // Aloca espaço para dadosCarro e sharedData
    ftruncate(msID, sizeof(carData));  
    memory = mmap(NULL, sizeof(carData), PROT_READ | PROT_WRITE, MAP_SHARED, msID, 0);

    if (memory == MAP_FAILED) {
        perror("mmap falhou");
        exit(1);
    }

    // Inicializa o exit_flag
    memory->exitFlag = 0; 
    memory->adasQTD = 0;
    memory->atQTD = 0;

    // Criação dos semáforos
    sem_unlink("/semaA");
    sem_unlink("/semaB");
    sem_unlink("/semC");

    semA = sem_open("/semA", O_CREAT | O_EXCL, 0666, 1); 
    semB = sem_open("/semB", O_CREAT | O_EXCL, 0666, 0);
    semC = sem_open("/semC", O_CREAT | O_EXCL, 0666, 0);

    if (semA == SEM_FAILED || semB == SEM_FAILED || semC == SEM_FAILED) {
        perror("Erro ao criar semáforos");
        cleanup();
        exit(1);
    }

    // Criação da fila
    struct mq_attr attr;
    attr.mq_flags = 0;
    attr.mq_maxmsg = 10;  
    attr.mq_msgsize = 1; 
    attr.mq_curmsgs = 0;

    mq_unlink("/commandQueue");

    mqd_t mq = mq_open("/commandQueue", O_CREAT | O_WRONLY, 0666, &attr);

    if (mq == -1) {
        perror("Falha ao abrir fila");
        exit(1);
    }

    mq_close(mq);

}



























