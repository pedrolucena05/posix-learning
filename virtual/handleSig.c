#include "handleSig.h"

// Manipulador de sinal para o processo principal
void handleSignal(int sig) {
    printf("\n(Signal) Recebido sinal SIGUSR2. Limpando recursos e encerrando processos...\n");
    memory->exitFlag = 1; // Atualiza o exit_flag na memória compartilhada
}

void handlePause(int sig) {
    printf("\n(Signal) Recebido sinal SIGUSR1. Pressione enter para continuar...\n");
    getchar(); 
}