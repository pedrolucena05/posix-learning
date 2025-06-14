#include "controller.h"

void ADAS () {
    if (memory->carVelocity > 120) {
        printf("\n(Controller) (ADAS) Veiculo ultrapassou a velocidade maxima! desacelerando o carro...");
        memory->carVelocity = 120;
        memory->adasQTD++;
    }
    else if (memory->carVelocity < 60) {
        printf("\n(Controller) (ADAS) veiculo ultrapassou o limite minimo de velocidade! Acelerando o veiculo...");
        memory->carVelocity = 60;    
        memory->adasQTD++;
    }
}

void executeComands (char msg) {
    
    switch (msg) {

        case '1':
            printf("\n(Controller) Acelerando o veículo...");
            memory->carVelocity += 10;
            memory->atQTD++;
            ADAS();
            break;

        case '2':
            printf("\n(Controller) Freio acionado!");
            memory->carVelocity -= 10;
            memory->atQTD++;
            ADAS();
            break;

        case '3':
            printf("\n(Controller) Seta para a esquerda ligada.");
            memory->atQTD++;
            break;

        case '4':
            printf("\n(Controller) Seta para a direita ligada.");
            memory->atQTD++;
            break;

        case '5':
            printf("\n(Controller) Farol alto ligado.");
            memory->atQTD++;
            break;

        case '6':
            printf("\n(Controller) Farol baixo ligado.");
            memory->atQTD++;
            break;
              
        default:
            
            break;
    }
}

void controllerProcess() {
    
    mqd_t mqR;
    char msg;
    unsigned char flag = 0;
  
    // Loop infinito até que a condição de parada seja atendida
    while (memory->exitFlag == 0) {  
        sem_wait(semC);

        if (memory->exitFlag != 0) {
            sem_post(semA);
            break;        
        }
        
        if (flag != 0) {
            mq_receive(mqR, &msg, 1, NULL);
        }

        flag = 1;        

        printf("\n(Controller) Dados da memoria compartilhada antes da manipulacao do controlador:\n");
        printf("(Controller) Velocidade: %d | Rotação do motor: %d | Temperatura do motor: %d", memory->carVelocity, memory->enRotation, memory->enTemp);

        ADAS();
        
        mqR = mq_open("/commandQueue", O_RDONLY | O_NONBLOCK);

        if (mqR == -1) {
            perror("(Controller) Falha ao abrir fila");
            exit(1);
        }

                
                
        while (mq_receive(mqR, &msg, 1, NULL) != -1) {
            executeComands(msg);
        }
                        
        mq_close(mqR);
        sleep(1);
        printf("\n\n");
        sem_post(semA);
    }
}

