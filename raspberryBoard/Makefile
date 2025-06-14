# Nome do executável
TARGET = main

# Arquivos fonte
SRCS = main.c

# Compilador
CC = gcc

# Flags do compilador
CFLAGS = -Wall -Wextra -O2

# Bibliotecas
LIBS = -lpthread -lwiringPi -lrt

# Regra principal
all: $(TARGET)

# Regra para gerar o executável
$(TARGET): $(SRCS)
	$(CC) $(CFLAGS) -o $(TARGET) $(SRCS) $(LIBS)

# Limpeza dos arquivos gerados
clean:
	rm -f $(TARGET)

