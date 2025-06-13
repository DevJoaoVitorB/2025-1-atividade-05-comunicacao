#include <stdio.h>
#include <stdlib.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <unistd.h>

#define SHM_SIZE 1024

int main() {
    int shmid;
    key_t key = 1234;  // Mesma chave usada no escritor
    char *shm;

    // Obtém o segmento de memória compartilhada
    shmid = shmget(key, SHM_SIZE, 0666);
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

    // Lê a mensagem da memória compartilhada
    printf("Leitor: Mensagem lida: \"%s\"\n", shm);

    // Sinaliza para o escritor que terminou (modificando o primeiro byte)
    *shm = '*';

    // Libera a memória compartilhada
    shmdt(shm);

    printf("Leitor: Finalizado.\n");
    return 0;
}
