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
    
    // Criar área de memória compartilhada
    fd = shm_open(SHARED_MEM_NAME, O_CREAT | O_RDWR, 0666);
    if (fd == -1) {
        perror("shm_open");
        exit(EXIT_FAILURE);
    }
    
    // Ajustar o tamanho da memória compartilhada
    if (ftruncate(fd, sizeof(shared_data)) == -1) {
        perror("ftruncate");
        exit(EXIT_FAILURE);
    }
    
    // Mapear a memória compartilhada
    shared = mmap(NULL, sizeof(shared_data), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (shared == MAP_FAILED) {
        perror("mmap");
        exit(EXIT_FAILURE);
    }
    
    // Inicializar mutex e condition variable com atributos para compartilhamento entre processos
    pthread_mutexattr_t mutex_attr;
    pthread_condattr_t cond_attr;
    
    pthread_mutexattr_init(&mutex_attr);
    pthread_mutexattr_setpshared(&mutex_attr, PTHREAD_PROCESS_SHARED);
    pthread_mutex_init(&shared->mutex, &mutex_attr);
    
    pthread_condattr_init(&cond_attr);
    pthread_condattr_setpshared(&cond_attr, PTHREAD_PROCESS_SHARED);
    pthread_cond_init(&shared->cond, &cond_attr);
    
    shared->ready = 0;
    
    // Escrever a mensagem
    pthread_mutex_lock(&shared->mutex);
    strncpy(shared->message, "Olá, comunicação via memória compartilhada com threads!", MESSAGE_SIZE);
    shared->ready = 1;
    printf("Writer: Mensagem escrita. Notificando leitor...\n");
    pthread_cond_signal(&shared->cond);
    pthread_mutex_unlock(&shared->mutex);
    
    // Esperar pela confirmação de leitura
    pthread_mutex_lock(&shared->mutex);
    while (shared->ready != 2) {
        pthread_cond_wait(&shared->cond, &shared->mutex);
    }
    printf("Writer: Confirmação recebida. Mensagem lida pelo leitor.\n");
    pthread_mutex_unlock(&shared->mutex);
    
    // Limpeza
    munmap(shared, sizeof(shared_data));
    shm_unlink(SHARED_MEM_NAME);
    
    return 0;
}
