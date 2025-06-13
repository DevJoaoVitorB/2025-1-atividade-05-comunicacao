# Imagem base Fedora
FROM fedora:latest

# Atualiza e instala GCC e terminal fish
RUN dnf -y update && \
    dnf -y install gcc fish && \
    dnf clean all

# Cria diretório padrão da aplicação
RUN mkdir -p /app

# Define o diretório de trabalho
WORKDIR /app

# Comando padrão: entra no shell fish (como o professor mostrou)
CMD ["fish"]