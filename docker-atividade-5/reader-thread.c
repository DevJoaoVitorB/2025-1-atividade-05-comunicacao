#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/mman.h>
#include <fcntl.h>

#define SHARED_MEM_NAME "/shared_mem_thread"
#define MESSAGE_SIZE 1024

typedef struct {
    pthread_mutex_t mutex;
    pthread_cond_t cond;
    char message[MESSAGE_SIZE];
    int ready;
} shared_data;

int main() {
    int fd;
    shared_data *shared;
    
    // Abrir área de memória compartilhada
    fd = shm_open(SHARED_MEM_NAME, O_RDWR, 0666);
    if (fd == -1) {
        perror("shm_open");
        exit(EXIT_FAILURE);
    }
    
    // Mapear a memória compartilhada
    shared = mmap(NULL, sizeof(shared_data), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (shared == MAP_FAILED) {
        perror("mmap");
        exit(EXIT_FAILURE);
    }
    
    // Esperar pela mensagem
    pthread_mutex_lock(&shared->mutex);
    while (shared->ready != 1) {
        pthread_cond_wait(&shared->cond, &shared->mutex);
    }
    
    // Ler a mensagem
    printf("Reader: Mensagem recebida: %s\n", shared->message);
    
    // Confirmar leitura
    shared->ready = 2;
    pthread_cond_signal(&shared->cond);
    pthread_mutex_unlock(&shared->mutex);
    
    // Limpeza
    munmap(shared, sizeof(shared_data));
    close(fd);
    
    return 0;
}
