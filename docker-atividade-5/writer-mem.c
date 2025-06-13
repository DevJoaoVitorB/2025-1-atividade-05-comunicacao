#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <unistd.h>

#define SHM_SIZE 1024  // Tamanho da memória compartilhada

int main() {
    int shmid;
    key_t key = 1234;  // Chave única para a memória compartilhada
    char *shm, *data;

    // Cria um segmento de memória compartilhada
    shmid = shmget(key, SHM_SIZE, IPC_CREAT | 0666);
    if (shmid == -1) {
        perror("shmget");
        exit(1);
    }

    // Anexa o segmento ao espaço de endereçamento do processo
    shm = shmat(shmid, NULL, 0);
    if (shm == (char *)-1) {
        perror("shmat");
        exit(1);
    }

    // Escreve na memória compartilhada
    strcpy(shm, "Olá, Memória Compartilhada!");

    printf("Escritor: Mensagem escrita na memória compartilhada.\n");
    printf("Escritor: Aguardando leitura...\n");

    // Espera até que o leitor sinalize que terminou (modificando o primeiro byte)
    while (*shm != '*') {
        sleep(1);
    }

    // Libera a memória compartilhada
    shmdt(shm);
    shmctl(shmid, IPC_RMID, NULL);  // Remove o segmento

    printf("Escritor: Memória compartilhada liberada.\n");
    return 0;
}
