#include <stdio.h>
#include "garfo.h"

//teste

int main() {
    // Lê o grafo ponderado do arquivo
    GrafoP *g = le_grafo_pesos("grafo_W_1.txt");

    // Vetores auxiliares
    float dist[g->n];
    int pai[g->n];

    // Executa Dijkstra a partir do vértice 10
    dijkstra_vetor(g, 9, dist, pai);  // vértice 10 → índice 9 (base 0)

    // Exibe o resultado para 20 e 30
    printf("Distância mínima de 10 para 20 = %.3f\n", dist[19]);
    printf("Caminho 10 -> 20: ");
    imprime_caminho_p(pai, 9, 19);
    printf("\n\n");

    printf("Distância mínima de 10 para 30 = %.3f\n", dist[29]);
    printf("Caminho 10 -> 30: ");
    imprime_caminho_p(pai, 9, 29);
    printf("\n");

    printf("Distância mínima de 10 para 40 = %.3f\n", dist[29]);
    printf("Caminho 10 -> 40: ");
    imprime_caminho_p(pai, 9, 39);
    printf("\n");

    printf("Distância mínima de 10 para 50 = %.3f\n", dist[9]);
    printf("Caminho 10 -> 50: ");
    imprime_caminho_p(pai, 9, 49);
    printf("\n");

    printf("Distância mínima de 10 para 60 = %.3f\n", dist[59]);
    printf("Caminho 10 -> 60: ");
    imprime_caminho_p(pai, 9, 59);
    printf("\n");

    // Libera a memória
    libera_grafo_p(g);

    return 0;
    }
