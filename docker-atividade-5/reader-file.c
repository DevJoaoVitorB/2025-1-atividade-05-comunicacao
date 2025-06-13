#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define FILENAME "comunicacao.txt"
#define TEMPNAME "comunicacao.lida"

int main() {
    FILE *file;
    char buffer[1024];

    // Abre o arquivo para leitura
    file = fopen(FILENAME, "r");
    if (file == NULL) {
        perror("fopen");
        exit(1);
    }

    // Lê o conteúdo do arquivo
    fread(buffer, 1, sizeof(buffer), file);
    fclose(file);

    printf("Leitor: Mensagem lida:\n%s", buffer);

    // Remove o arquivo (sinaliza que terminou)
    if (rename(FILENAME, TEMPNAME) != 0) {
        perror("rename");
        exit(1);
    }

    printf("Leitor: Arquivo renomeado para %s. Finalizado.\n", TEMPNAME);
    return 0;
}
