#include "panel.h"

void panelProcess() {
    
    mqd_t mqW;
    char buffer[50];
    int i = 0;

    while (memory->exitFlag == 0) {  
        sem_wait(semB);

        if (memory->exitFlag != 0) {
            sem_post(semC);
            break;        
        }

        mqW = mq_open("/commandQueue", O_WRONLY);

        if (mqW == -1) {
            perror("Falha ao abrir fila");
            exit(1);
        }
        printf("\n(Panel) 1- Acelerar veiculo;\n(Panel) 2- Acionar freio;\n(Panel) 3- Ligar seta para esquerda;\n(Panel) 4- Ligar seta para direita;\n(Panel) 5- Ligar farol alto;\n(Panel) 6- Ligar farol baixo;\n(Panel) 7- Pausar aplicacao;\n(Panel) 0- Sair;");
        printf("\n(Panel) Conjunto de opcoes separadas por espacos: ");
        fgets(buffer, 50, stdin);    
        
        buffer[strcspn(buffer, "\n")] = 0;  
        
        char *token = strtok(buffer, " ");
        while (token != NULL && i < 9) {
            if (token[0] == '0') {
                pid_t pidC = getppid();

                if (kill(pidC, SIGUSR2) == -1) {
                    printf("\n(Controller) Erro ao mandar o sinal SIGUSR2");            
                }

                else {
                    break;                                            
                }                
            }
            
            if (token[0] == '7') {
               pid_t pidC = getppid();

               if (kill(pidC, SIGUSR1) == -1) {
                    printf("\n(Controller) Erro ao mandar o sinal SIGUSR1");            
                }

                else {
                    break;                                            
                }  
            }
            if (mq_send(mqW, &token[0], 1, 0) == -1) {
                perror("(Panel) Falha ao enviar mensagem");
                exit(1);
            }
            i++;
            token = strtok(NULL, " "); 
        }
        
        i = 0;

        mq_send(mqW, "t", 1, 0);
        mq_close(mqW);

        sleep(1);
        sem_post(semC);
    }   
}

