#include <stdio.h>
#include "garfo.h"

//teste

int main() {
    Grafo *g = le_grafo_arquivo("grafo_6.txt", 1); 

    if (!g) {
        printf("Erro ao ler grafo\n");
        return 1;
    }
    componentes_conexas(g, "componentes_saida.txt");
    libera_grafo(g);
    return 0;
}