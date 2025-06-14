#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <semaphore.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <signal.h>
#include <time.h>
#include <sys/wait.h>
#include <string.h>
#include <softPwm.h>
#include <wiringPi.h>
#include <pthread.h>

// Pinos utilizados
#define Pedal_FR 22 
#define Pedal_AC 27 
#define Farol_Baixo 19 
#define Farol_Alto 26 
#define Luz_Freio 25 
#define Luz_Seta_Esq 8 
#define Luz_Seta_Dir 7 
#define Luz_Temp_Motor 
#define Comando_Farol 16 
#define Comando_Farol_Alto 1 
#define Comando_seta_esq 20 
#define Comando_seta_dir 21 
#define CC_RES 13 
#define CC_CANCEL 0 
#define Motor_POT 23
#define Freio_INT 24
#define Motor_DIR1 17
#define Motor_DIR2 18
#define CC_RES 13
#define CC_CANCEL 0
#define Sensor_hall_roda_A 5

// Memória compartilhada
typedef struct {     
    int userInput; // serve para definir qual atuador será utilizado 
    int exitFlag; // condição de parada da aplicação com ctrl+c
    
} carData;

// Struct para definição manual dos pinos PWM
typedef struct {
    int acPIN;
    int acPower;
    int brPin;
    int brPower;
    int frequency;
} PwmParams;

volatile unsigned int pulse_count = 0;  // Contador de pulsos do hall a da roda
volatile double global_velocity = 0.0;  // Velocidade do veiculo
volatile struct timespec last_pulse_time;  // Tempo do último pulso de velocidade (para calculo de velocidade)
volatile struct timespec start_time; // Tempo do penúltimo pulso de velocidade (para calculo de velocidade)

PwmParams pwmParams;

// Ponteiro para a memória compartilhada
carData *memory;

// Semáforos para controle da alternância
sem_t *semA;
sem_t *semB;

// Vetor para configurar os pinos com PudUp e pudDown
int enterPins[] = {Comando_seta_esq, Comando_seta_dir, Comando_Farol, Comando_Farol_Alto, Pedal_FR, Pedal_AC, CC_RES, CC_CANCEL};
int pinsState[] = {1,1,1,1,1,1,1,1};

// Controle de atuação do cruise control
unsigned char cruiseControl = 0;
// Contorele de on/off dos atuadores (para que o comando seja feito apenas uma vez) 
unsigned char flagSE = 0, flagSD = 0, flagFA = 0, flagFB = 0, flagFR = 0, flagAC = 0, flagC = 0, flagCC = 0;
unsigned char contA = 0, contB = 0; // controle dos atuadores de aceleração e freios respectivamente

unsigned char enginePower = 0; 
unsigned char brakePower = 0;

unsigned char period = 0; // Periodo entre pulsos para calcular a velocidade

double lastVel = 0.0;

pthread_mutex_t pulse_mutex = PTHREAD_MUTEX_INITIALIZER; 

void signalHandler() {
    memory->exitFlag = 1; 

}

// Thread de calculo de velocidade
void *calculate_velocity() {
    struct timespec start_time, current_time;
    double period, frequency, velocity;
    double wheel_diameter = 0.616;  
    double wheel_circumference = 3.14 * wheel_diameter;  

    while (1) {
        pthread_mutex_lock(&pulse_mutex);  
        int local_pulse_count = pulse_count;  
        pthread_mutex_unlock(&pulse_mutex);

        if (local_pulse_count % 2 == 1) {
            clock_gettime(CLOCK_MONOTONIC, &start_time);  // Pega o tempo inicial
        } else {
            clock_gettime(CLOCK_MONOTONIC, &current_time);  // Pega o tempo atual
            period = (current_time.tv_sec - start_time.tv_sec) + (current_time.tv_nsec - start_time.tv_nsec) / 1e9;

            // Verifica se o período é válido
            if (period > 0) {
                frequency = 1.0 / period;
                velocity = wheel_circumference * frequency * 3.6 * 2;  // Velocidade em km/h

                pthread_mutex_lock(&pulse_mutex);
                global_velocity = velocity;  // Atualiza a velocidade global
                pthread_mutex_unlock(&pulse_mutex);
            }
        }

        // Reinicia o contador de pulsos 
        pthread_mutex_lock(&pulse_mutex);
        if (pulse_count >= 9998) {
            pulse_count = 0;
        }
        pthread_mutex_unlock(&pulse_mutex);
    }

    return NULL;
}

// Função de callback (incremento de pulsos)
void pulse_callback(void) {
    pthread_mutex_lock(&pulse_mutex);
    pulse_count++;
    pthread_mutex_unlock(&pulse_mutex);
}

// Thread de cálculo manual dos pinos PWM
void* manualPwmControl(void* arg) {
    PwmParams* params = (PwmParams*)arg;  // Convertendo void* para PwmParams*

    int period = 1000000 / params->frequency; // Período em microssegundos (us)

    while (1) {
        int onTime1 = (params->acPower * period) / 100;
        int onTime2 = (params->brPower * period) / 100;
        int offTime1 = period - onTime1;
        int offTime2 = period - onTime2;

        // PWM no pino 1
        digitalWrite(params->acPIN, HIGH);
        usleep(onTime1); // Ligado
        digitalWrite(params->acPIN, LOW);
        usleep(offTime1); // Desligado

        // PWM no pino 2
        digitalWrite(params->brPin, HIGH);
        usleep(onTime2); // Ligado
        digitalWrite(params->brPin, LOW);
        usleep(offTime2); // Desligado
        usleep(1000); 
    }
    return NULL;
}

// Função para o processo filho: simula entrada de comando do usuário
void panelProcess() {

    // Configuração dos pinos de entrada
    pinMode(Comando_seta_esq, INPUT);
    pinMode(Comando_seta_dir, INPUT);
    pinMode(Comando_Farol, INPUT);
    pinMode(Comando_Farol_Alto, INPUT);
    pinMode(Pedal_FR, INPUT);
    pinMode(Pedal_AC, INPUT);

    for(int i = 0; i < 8; i++) {

        if(digitalRead(enterPins[i]) == 0) {
            pullUpDnControl(enterPins[i], PUD_DOWN);             
        }
        else {
            pullUpDnControl(enterPins[i], PUD_UP);
            pinsState[i] = 0;
        }
    }

    delay(500);
    
    // verificação de valor dos pinos quando estão desligados
    printf("\nSETA ESQ: %d, ST= %d", digitalRead(Comando_seta_esq), pinsState[0]);
    printf("\nSETA DIR: %d, ST= %d", digitalRead(Comando_seta_dir), pinsState[1]);
    printf("\nFAROL BAIXO: %d, ST= %d", digitalRead(Comando_Farol), pinsState[2]);
    printf("\nFAROL ALTO: %d, ST= %d", digitalRead(Comando_Farol_Alto), pinsState[3]);
    printf("\nFREIO: %d, ST= %d", digitalRead(Pedal_FR), pinsState[4]);
    printf("\nAcelerador: %d, ST= %d", digitalRead(Pedal_AC), pinsState[5]);
    printf("\nCruise Control: %d, ST= %d", digitalRead(CC_RES), pinsState[6]);
    printf("\nCancel CC: %d, ST= %d\n", digitalRead(CC_CANCEL), pinsState[7]);

    delay(500);

    printf("\n\nLoop do processo de captação de sensores ativado");


    while (memory->exitFlag == 0) {
        sem_wait(semB);  

        usleep(1000);  // Pequeno atraso para evitar "polling" excessivo
        
        // Condição de parada gerado pelo signal handler (ctrl+c)
        if (memory->exitFlag != 0) {
            sem_post(semA);  
            break;
        }
        
        

        // Detecta o acionamento do botão da seta esquerda
        if (digitalRead(Comando_seta_esq) == pinsState[0] && flagSE == 0) {
            memory->userInput = 20;  // Sinal para iverter o valor do atuador no controllerProcess
            printf("\nMandando sinal para modificar seta esquerda"); 
            flagSE = 1;
            
           
        }

        // Detecta o acionamento do botão da seta direita
        else if (digitalRead(Comando_seta_dir) == pinsState[1] && flagSD == 0) {
            memory->userInput = 21;  
            printf("\nMandando sinal para modificar seta direita"); 
            flagSD = 1;
            
        }


        // Detecta o acionamento do botão do farol baixo
        else if (digitalRead(Comando_Farol) == pinsState[2] && flagFB == 0) {
            memory->userInput = 16;  
            printf("\nMandando sinal para modificar farol baixo"); 
            flagFB = 1;
            
            
        } 

        // Detecta o acionamento do botão do farol alto
        else if (digitalRead(Comando_Farol_Alto) == pinsState[3] && flagFA == 0) {
            memory->userInput = 1;  
            printf("\nMandando sinal para modificar farol alto"); 
            flagFA = 1;
            
        } 

        // Detecta o acionamento do botão de freio
        else if (digitalRead(Pedal_FR) == pinsState[4] && flagFR == 0) {
            memory->userInput = 22;  
            printf("\nMandando sinal para modificar luz freio"); 
            flagFR = 1;
            
        }

        // Detecta o acionamento do botão de aceleração
        else if (digitalRead(Pedal_AC) == pinsState[5] && flagAC == 0) {
            memory->userInput = 27;
            printf("\nMandando sinal para acelerar o veiculo");
            flagAC = 1; 
        }

        // Detecta o acionamento do botão de cruise control
        else if (digitalRead(CC_RES) == pinsState[6] && flagC == 0) {
            memory->userInput = 13;
            printf("\nMandando sinal para ativar o cruise control");
            flagC = 1;
        }

        // Detecta o acionamento do botão de cancel do cruise control 
        else if (digitalRead(CC_CANCEL) == pinsState[7] && flagCC == 0) {
            memory->userInput = 0;
            printf("\nMandando sinal para cancelar o cruise control");
            flagCC = 1;
        }

        // Detecta que o atuador já foi acionado (evita multiplos envios de acionamento)
        if (digitalRead(CC_RES) == !pinsState[6] && flagC == 1) {
            flagC = 0;
        }
        if (digitalRead(CC_CANCEL) == !pinsState[7] && flagCC == 1) {
            flagCC = 0;
        }

        if (digitalRead(Comando_seta_esq) == !pinsState[0] && flagSE == 1) {
            flagSE = 0;
            
        }
        else if (digitalRead(Comando_seta_dir) == !pinsState[1] && flagSD == 1) {
            flagSD = 0;
                    
        }
        else if (digitalRead(Comando_Farol) == !pinsState[2] && flagFB == 1) {
            flagFB = 0;
                    
        }
        else if (digitalRead(Comando_Farol_Alto) == !pinsState[3] && flagFA == 1) {
            flagFA = 0;
                    
        }
        else if (digitalRead(Pedal_FR) == !pinsState[4] && flagFR == 1) {
            flagFR = 0;
                    
        }
        else if (digitalRead(Pedal_AC) == !pinsState[5] && flagAC == 1) {
            flagAC = 0;
                   
        }
                

        sem_post(semA);
        
    }

    printf("[FILHO] Finalizando...\n\n");
}


void controllerProcess() {

    printf("\n\nLoop do processo de atuadores inicializado");

    while (memory->exitFlag == 0) {
        sem_wait(semA); 
        
         

        if (memory->exitFlag != 0) {
            sem_post(semB);
            break;
        }
        
        // Verifica o comando do usuário e inverte os sinais correspondentes
        if (memory->userInput == 20) {
            
            digitalWrite(Luz_Seta_Esq, !digitalRead(Luz_Seta_Esq));
            if (digitalRead(Luz_Seta_Esq) == HIGH) {
                digitalWrite(Luz_Seta_Dir, LOW);                
            }
            printf("\nAtivando atuador de seta para esquerda");
            
            memory->userInput = -1;
        }

        else if (memory->userInput == 21) {
            digitalWrite(Luz_Seta_Dir, !digitalRead(Luz_Seta_Dir));
            if (digitalRead(Luz_Seta_Dir) == HIGH) {
                digitalWrite(Luz_Seta_Esq, LOW);          
            }
            printf("\nAtivando atuador de seta para direita");
            memory->userInput = -1;
        }

        else if (memory->userInput == 16) {
            digitalWrite(Farol_Baixo, !digitalRead(Farol_Baixo));
            if (digitalRead(Farol_Baixo) == HIGH) {
                digitalWrite(Farol_Alto, LOW);                
            }
            printf("\nAtivando atuador de farol baixo"); 
            memory->userInput = -1;
        }

        else if (memory->userInput == 1) {
            digitalWrite(Farol_Alto, !digitalRead(Farol_Alto));
            if (digitalRead(Farol_Alto) == HIGH) {
                digitalWrite(Farol_Baixo, LOW);    
            }
            printf("\nAtivando atuador de farol alto");
            memory->userInput = -1;
        }

        else if (memory->userInput == 22) {
            digitalWrite(Luz_Freio, !digitalRead(Luz_Freio));
            
            contB++;
            if (contB == 1) {
                pwmParams.acPower = enginePower;
                usleep(1000);             
                pwmParams.brPower = 50;
                usleep(1000);  
                brakePower = 50;
                digitalWrite(Motor_DIR1, HIGH);
                digitalWrite(Motor_DIR2, HIGH); 
                
            }
            else if (contB == 2) {
                digitalWrite(Motor_DIR1, HIGH);
                digitalWrite(Motor_DIR2, LOW);
                pwmParams.acPower = enginePower;
                usleep(1000);             
                pwmParams.brPower = 0;
                usleep(1000);  
                brakePower = 0;
                contB = 0; 
            } 
                                    
            printf("\nAtivando atuador de freio"); 
                
            memory->userInput = -1;
        }

        else if (memory->userInput == 27) {
            printf("\nAtivando atuador da Aceleracao");
            contA++;
            if (contA == 1) {
                pwmParams.acPower = 100;
                usleep(1000);             
                pwmParams.brPower = brakePower;
                enginePower = 100;
                usleep(1000);        
                digitalWrite(Motor_DIR1, HIGH);
                digitalWrite(Motor_DIR2, LOW);
                
            }
            else if (contA == 2) {
                softPwmWrite(Motor_POT, 0);
                pwmParams.acPower = 0;
                enginePower = 0;
                usleep(1000);
                pwmParams.brPower = brakePower;
                usleep(1000);    
                contA = 0; 
            } 
            memory->userInput = -1; 
        }

        else if (memory->userInput == 13) {
            cruiseControl = 1;
            printf("\nAtivando cruise control");        
            memory->userInput = -1;              
        }

        else if (memory->userInput == 0){
            digitalWrite(Motor_DIR1, HIGH);
            digitalWrite(Motor_DIR2, LOW);
            pwmParams.acPower = 0;             
            pwmParams.brPower = 0;  
            cruiseControl = 0;
            printf("\nCancelando cruise control");
            memory->userInput = -1;
         
        }


        if (cruiseControl == 1) {
            pthread_mutex_lock(&pulse_mutex);
            double cruiseVel = global_velocity;
            pthread_mutex_unlock(&pulse_mutex);
            if (lastVel == cruiseVel) {
                lastVel = cruiseVel;
                if (cruiseVel < 35.0) {
                    digitalWrite(Motor_DIR1, HIGH);
                    digitalWrite(Motor_DIR2, LOW);      
                    pwmParams.acPower = 100;
                    usleep(500);             
                    pwmParams.brPower = 0;
                    usleep(500);
                    enginePower = 100;
                    brakePower = 0;                   
                }
                else if (cruiseVel > 55.0){
                    pwmParams.acPower = 0;
                    usleep(500);             
                    pwmParams.brPower = 20;
                    usleep(500);
                    enginePower = 0;
                    brakePower = 20;               
                }
                else if (cruiseVel < 40.0 && cruiseVel >= 35.0) {
                    pwmParams.acPower = 20;
                    usleep(500);             
                    pwmParams.brPower = 0;
                    usleep(500);
                    enginePower = 20;
                    brakePower = 0;                  
                }
                else if (cruiseVel <= 55.0 && cruiseVel >= 45.0) {
                    pwmParams.acPower = 0;
                    usleep(500);             
                    pwmParams.brPower = 10;
                    usleep(500);
                    enginePower = 0;
                    brakePower = 10;                   
                }
                else if (cruiseVel <= 45.0 && cruiseVel >= 40.0) {
                    pwmParams.acPower = 0;
                    usleep(500);             
                    pwmParams.brPower = 0;
                    usleep(500);
                    enginePower = 0;
                    brakePower = 0;  
                }
            }
        }    
        pthread_mutex_lock(&pulse_mutex);
        lastVel = global_velocity;
        pthread_mutex_unlock(&pulse_mutex);

        sem_post(semB);
    }

    printf("[PAI] Finalizando...\n");
}

int main() {

    int mID;
    pid_t panel;
    pthread_t pwmThread;
    

    if (wiringPiSetupGpio() == -1) {
        perror("Erro ao configurar wiringPi");
        exit(1);
    }
    
    // Configuração de memória compartilhada
    shm_unlink("/memory");
    mID = shm_open("/memory", O_CREAT | O_RDWR, 0666);
    ftruncate(mID, sizeof(carData));
    memory = mmap(0, sizeof(carData), PROT_READ | PROT_WRITE, MAP_SHARED, mID, 0);

    // Inicializa a memória compartilhada
    memory->userInput = -1;
    memory->exitFlag = 0;

    // Configuração dos semáforos
    sem_unlink("/semA");
    sem_unlink("/semB");
    semA = sem_open("/semA", O_CREAT, 0666, 0);
    semB = sem_open("/semB", O_CREAT, 0666, 1);

    printf("\n(Main) aplicação inicializada!");

    if (mID == -1) {
        perror("Erro ao criar memória compartilhada");
        exit(1);
    }

    if (semA == SEM_FAILED || semB == SEM_FAILED) {
        perror("Erro ao criar semáforos");
        exit(1);
    }

    // Cria o ISR para calculo de velocidae
    pinMode(Sensor_hall_roda_A, INPUT);
    if(digitalRead(Sensor_hall_roda_A) == 0) {
        pullUpDnControl(Sensor_hall_roda_A, PUD_DOWN);             
    }
    else {
        pullUpDnControl(Sensor_hall_roda_A, PUD_UP);
            
    }
    // definição da função calback do ISR
    if (wiringPiISR(Sensor_hall_roda_A, INT_EDGE_RISING, &pulse_callback) < 0) {
        fprintf(stderr, "Erro ao configurar o ISR.\n");
        return 1;
    }
    
    // cria thread de cálculo de velocidade
    pthread_t velocity_thread;
    if (pthread_create(&velocity_thread, NULL, calculate_velocity, NULL) != 0) {
        fprintf(stderr, "Erro ao criar a thread.\n");
        return 1;
    } 

    // Configura o sinal para CTRL+C
    signal(SIGINT, signalHandler);

    // Definição de potencia de aceleração e freio
    pwmParams.acPIN = Motor_POT;
    pwmParams.acPower = 50;
    pwmParams.brPin = Freio_INT;
    pwmParams.brPower = 30;
    pwmParams.frequency = 100;
    
    // cria thread de calculo de valores dos pinos PWM
    pthread_create(&pwmThread, NULL, manualPwmControl, (void*)&pwmParams);

    
    pinMode(Luz_Seta_Esq, OUTPUT);
    pinMode(Luz_Seta_Dir, OUTPUT);
    pinMode(Farol_Baixo, OUTPUT);
    pinMode(Farol_Alto, OUTPUT);
    pinMode(Luz_Freio, OUTPUT);
    pinMode(Motor_DIR1, OUTPUT);
    pinMode(Motor_DIR2, OUTPUT);
    pinMode(Motor_POT, OUTPUT);
    pinMode(Freio_INT, OUTPUT);  


    digitalWrite(Luz_Seta_Esq, LOW);
    digitalWrite(Luz_Seta_Dir, LOW);
    digitalWrite(Farol_Baixo, LOW);
    digitalWrite(Farol_Alto, LOW);
    digitalWrite(Luz_Freio, LOW);
    digitalWrite(Motor_DIR1, LOW);
    digitalWrite(Motor_DIR2, LOW);
    
    // Inicializa os valores dos pinos PWM
    pwmParams.acPower = 100;  
    pwmParams.brPower = 100; 
    delay(1000);

    printf("Aguardando estabilização dos pinos...\n");
    usleep(200000);

    // Criação do processo panel
    panel = fork();

    if (panel == 0) {
        printf("\nEntrando no loop do processo filho");
        // Processo filho
        panelProcess();
    } 
     
    else {
        printf("\nEntrando no loop do processo pai");

        controllerProcess();

        wait(NULL);

        // Limpeza
        sem_close(semA);
        sem_close(semB);
        sem_unlink("/semA");
        sem_unlink("/semB");
        munmap(memory, sizeof(carData));
        shm_unlink("/memory");
        close(mID);
    }

    digitalWrite(Motor_DIR1, LOW);
    digitalWrite(Motor_DIR2, LOW); 
    
    pthread_cancel(pwmThread);
    pthread_join(pwmThread, NULL);

    pthread_cancel(velocity_thread);
    

    pthread_join(velocity_thread, NULL);
   
    printf("\nAplicacao finalizada com sucesso");
    return 0;
}
