#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>      // Para open(), O_CREAT, etc.
#include <sys/stat.h>   // Para permissões (S_IRUSR, S_IWUSR)

#define FILENAME "comunicacao.txt"

int main() {
    FILE *file;
    const char *message = "Olá, comunicação via arquivo!\n";

    // Abre o arquivo para escrita (cria se não existir, limpa conteúdo existente)
    file = fopen(FILENAME, "w");
    if (file == NULL) {
        perror("fopen");
        exit(1);
    }

    // Escreve a mensagem no arquivo
    fwrite(message, 1, strlen(message), file);
    fclose(file);

    printf("Escritor: Mensagem escrita no arquivo.\n");
    printf("Escritor: Aguardando leitura...\n");

    // Espera até que o leitor termine (verifica se o arquivo foi renomeado)
    while (access(FILENAME, F_OK) == 0) {
        sleep(1);
    }

    printf("Escritor: Arquivo lido e removido. Finalizado.\n");
    return 0;
}
