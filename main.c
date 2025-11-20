#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include "garfo.h"

// imprime caminho usando pai[] gerado por Bellman–Ford reverso
void imprime_caminho_bf(int *pai, int destino, int atual);

int main() {

    // ==========================================================
    // 1. Carregar grafo
    // ==========================================================
    Grafo *g = le_grafo("grafo_W_2.txt",1,1);
    if (!g) {
        printf("Erro ao carregar grafo.\n");
        return 1;
    }

    int destino = 100;  // vértice t (base 1)

    // ==========================================================
    // 2. Medir tempo do Bellman–Ford
    // ==========================================================
    clock_t t0 = clock();
    ResultadoBF R = bellman_ford(g, destino);
    clock_t t1 = clock();

    double tempo = (double)(t1 - t0) / CLOCKS_PER_SEC;

    printf("\nTempo de execução do Bellman–Ford: %.6f segundos\n\n", tempo);

    // ==========================================================
    // 3. Detectar ciclo negativo
    // ==========================================================
    if (R.negativo)
        printf("ATENÇÃO: Ciclo negativo detectado!\n\n");
    else
        printf("Nenhum ciclo negativo encontrado.\n\n");

    // ==========================================================
    // 4. Consultar distâncias 10, 20, 30 → 100
    // ==========================================================
    int consultas[3] = {10, 20, 30};

    for (int i = 0; i < 3; i++) {
        int u = consultas[i] - 1;   // base 0
        float d = R.dist[u];

        printf("Distância %d → %d = ", consultas[i], destino);
        if (d == INFINITY) printf("infinito (sem caminho)\n");
        else printf("%.3f\n", d);

        printf("Caminho: ");
        imprime_caminho_bf(R.pai, destino - 1, u);
        printf("\n\n");
    }

    // ==========================================================
    // 5. Liberar memória
    // ==========================================================
    free(R.dist);
    free(R.pai);
    libera_grafo(g);

    return 0;
}

