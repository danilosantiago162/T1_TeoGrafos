#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include "garfo.h"

//teste

// ---------- Criação ----------
Grafo* cria_grafo(int n, int representacao) {
    Grafo *g = malloc(sizeof(Grafo));   
    if (!g) return NULL;

    g->n = n;
    g->m = 0;
    g->representacao = representacao;
    g->grau = calloc(n, sizeof(int));

    if (representacao == 0) { // matriz
        g->matriz = malloc(n * sizeof(int*));
        for (int i = 0; i < n; i++) {
            g->matriz[i] = calloc(n, sizeof(int));
        }
        g->lista = NULL;
    } else { // lista
        g->lista = malloc(n * sizeof(int*));
        for (int i = 0; i < n; i++) {
            g->lista[i] = NULL;
        }
        g->matriz = NULL;
    }
    return g;
}

// ---------- Leitura ----------
Grafo* le_grafo_arquivo(const char *filename, int representacao) {
    FILE *f = fopen(filename, "r");
    if (!f) { perror("Erro ao abrir arquivo"); exit(1); }

    int n, u, v;
    fscanf(f, "%d", &n);
    Grafo *g = cria_grafo(n, representacao);

    while (fscanf(f, "%d %d", &u, &v) == 2) {
        u--; v--; // índice base 0
        g->m++;
        g->grau[u]++; g->grau[v]++;

        if (representacao == 0) {
            g->matriz[u][v] = g->matriz[v][u] = 1;
        } else {
            // insere em lista de adjacência (simples)
            int *tmp = realloc(g->lista[u], (g->grau[u]) * sizeof(int));
            g->lista[u] = tmp;
            g->lista[u][g->grau[u]-1] = v;

            tmp = realloc(g->lista[v], (g->grau[v]) * sizeof(int));
            g->lista[v] = tmp;
            g->lista[v][g->grau[v]-1] = u;
        }
    }

    fclose(f);
    return g;
}

// ---------- Funções de grau ----------
int grau_min(Grafo *g) {
    int min = INT_MAX;
    for (int i = 0; i < g->n; i++)
        if (g->grau[i] < min) min = g->grau[i];
    return min;
}

int grau_max(Grafo *g) {
    int max = 0;
    for (int i = 0; i < g->n; i++)
        if (g->grau[i] > max) max = g->grau[i];
    return max;
}

double grau_medio(Grafo *g) {
    int soma = 0;
    for (int i = 0; i < g->n; i++) soma += g->grau[i];
    return (double)soma / g->n;
}

double grau_mediana(Grafo *g) {
    int *copia = malloc(g->n * sizeof(int));
    if (!copia) return 0.0; 
    memcpy(copia, g->grau, g->n * sizeof(int));
    for (int i = 0; i < g->n-1; i++)
        for (int j = i+1; j < g->n; j++)
            if (copia[i] > copia[j]) {
                int tmp = copia[i]; copia[i] = copia[j]; copia[j] = tmp;
            }
    double mediana = (g->n % 2) ? copia[g->n/2] :
                     (copia[g->n/2 - 1] + copia[g->n/2]) / 2.0;
    free(copia);
    return mediana;
}

// ---------- Saída ----------
void salva_informacoes(const char *filename, Grafo *g) {
    FILE *f = fopen(filename, "w");
    if (!f) { perror("Erro ao salvar arquivo"); return; }

    fprintf(f, "Vertices: %d\n", g->n);
    fprintf(f, "Arestas: %d\n", g->m);
    fprintf(f, "Grau minimo: %d\n", grau_min(g));
    fprintf(f, "Grau maximo: %d\n", grau_max(g));
    fprintf(f, "Grau medio: %.2f\n", grau_medio(g));
    fprintf(f, "Mediana grau: %.2f\n", grau_mediana(g));

    fclose(f);
}
// ---------- BFS ----------
void bfs(Grafo *g, int inicio, const char *saida) {
    inicio--; // converter para índice base 0
    printf("Executando BFS a partir do vértice %d...\n", inicio);

    int *visitado = (int*) calloc(g->n, sizeof(int));
    int *pai = (int*) malloc(g->n * sizeof(int));
    int *nivel = (int*) malloc(g->n * sizeof(int));
    int *fila = (int*) malloc(g->n * sizeof(int));
    int frente = 0, tras = 0;

    for (int i = 0; i < g->n; i++) {
        pai[i] = -1;     // -1 = sem pai
        nivel[i] = -1;   // -1 = não visitado
    }

    // inicializa com o vértice inicial
    visitado[inicio] = 1;
    nivel[inicio] = 0;
    fila[tras++] = inicio;

    while (frente < tras) {
        int u = fila[frente++]; // desenfileira

        if (g->representacao == 0) { // matriz
            for (int v = 0; v < g->n; v++) {
                if (g->matriz[u][v] && !visitado[v]) {
                    visitado[v] = 1;
                    pai[v] = u;
                    nivel[v] = nivel[u] + 1;
                    fila[tras++] = v;
                }
            }
        } else { // lista
            for (int i = 0; i < g->grau[u]; i++) {
                int v = g->lista[u][i];
                if (!visitado[v]) {
                    visitado[v] = 1;
                    pai[v] = u;
                    nivel[v] = nivel[u] + 1;
                    fila[tras++] = v;
                }
            }
        }
    }

    // salva a árvore de busca no arquivo
    FILE *f = fopen(saida, "w");
    if (!f) { perror("Erro ao salvar BFS"); return; }

    fprintf(f, "BFS a partir do vertice %d\n", inicio+1);
    printf("BFS a partir do vertice %d\n", inicio+1);  

    for (int i = 0; i < g->n; i++) {
        fprintf(f, "Vertice %d: pai = %d, nivel = %d\n",
                i+1,
                (pai[i] == -1 ? -1 : pai[i]+1), // +1 pra voltar base 1
                nivel[i]);
    }

    fclose(f);

    free(visitado);
    free(pai);
    free(nivel);
    free(fila);
}

// ---------- DFS ----------
void dfs_visit(Grafo *g, int u, int *visitado, int *pai, int *nivel, int nivel_atual, FILE *f) {
    visitado[u] = 1;
    nivel[u] = nivel_atual;
    
    fprintf(f, "Vertice %d: pai = %d, nivel = %d\n",
            u+1,
            (pai[u] == -1 ? -1 : pai[u]+1),
            nivel[u]);

    printf("Vertice %d: pai = %d, nivel = %d\n",
            u+1,
            (pai[u] == -1 ? -1 : pai[u]+1),
            nivel[u]);

    if (g->representacao == 0) { // matriz
        for (int v = 0; v < g->n; v++) {
            if (g->matriz[u][v] && !visitado[v]) {
                pai[v] = u;
                dfs_visit(g, v, visitado, pai, nivel, nivel_atual + 1, f);
            }
        }
    } else { // lista
        for (int i = 0; i < g->grau[u]; i++) {
            int v = g->lista[u][i];
            if (!visitado[v]) {
                pai[v] = u;
                dfs_visit(g, v, visitado, pai, nivel, nivel_atual + 1, f);
            }
        }
    }
}

void dfs(Grafo *g, int inicio, const char *saida) {
    inicio--; // converter para índice base 0

    int *visitado = (int*) calloc(g->n, sizeof(int));
    int *pai = (int*) malloc(g->n * sizeof(int));
    int *nivel = (int*) malloc(g->n * sizeof(int));

    for (int i = 0; i < g->n; i++) {
        pai[i] = -1;
        nivel[i] = -1;
    }

    FILE *f = fopen(saida, "w");
    if (!f) { perror("Erro ao salvar DFS"); return; }

    fprintf(f, "DFS a partir do vertice %d\n", inicio+1);
    printf("DFS a partir do vertice %d\n", inicio+1);

    dfs_visit(g, inicio, visitado, pai, nivel, 0, f);

    fclose(f);

    free(visitado);
    free(pai);
    free(nivel);
}

int* bfs_distancias(Grafo *g, int inicio) {
    inicio--; // índice base 0
    int *dist = (int*) malloc(g->n * sizeof(int));
    for (int i = 0; i < g->n; i++) dist[i] = -1;

    int *fila = (int*) malloc(g->n * sizeof(int));
    int frente = 0, tras = 0;

    dist[inicio] = 0;
    fila[tras++] = inicio;

    while (frente < tras) {
        int u = fila[frente++];
        if (g->representacao == 0) { // matriz
            for (int v = 0; v < g->n; v++) {
                if (g->matriz[u][v] && dist[v] == -1) {
                    dist[v] = dist[u] + 1;
                    fila[tras++] = v;
                }
            }
        } else { // lista
            for (int i = 0; i < g->grau[u]; i++) {
                int v = g->lista[u][i];
                if (dist[v] == -1) {
                    dist[v] = dist[u] + 1;
                    fila[tras++] = v;
                }
            }
        }
    }

    free(fila);
    return dist; 
}

int distancia(Grafo *g, int u, int v) {
    int *dist = bfs_distancias(g, u);
    int d = dist[v-1]; // índice base 0
    free(dist);
    return d;
}
void componentes_conexas(Grafo *g, const char *saida) {
    FILE *f = fopen(saida, "w");
    if (!f) {
        printf("Erro ao abrir arquivo de saída\n");
        return;
    }

    int *visitado = (int*) calloc(g->n, sizeof(int));
    int componente = 0;

    for (int i = 0; i < g->n; i++) {
        if (!visitado[i]) {
            componente++;
            int qtd = 0;

            int *fila = (int) malloc(g->n * sizeof(int));
            int inicio = 0, fim = 0;

            fila[fim++] = i;
            visitado[i] = 1;

            while (inicio < fim) {
                int v = fila[inicio++];
                qtd++; // conta mais um vértice na componente

                if (g->representacao == 0) { // matriz
                    for (int j = 0; j < g->n; j++) {
                        if (g->matriz[v][j] && !visitado[j]) {
                            fila[fim++] = j;
                            visitado[j] = 1;
                        }
                    }
                } else { // lista
                    for (int k = 0; k < g->grau[v]; k++) {
                        int vizinho = g->lista[v][k];
                        if (!visitado[vizinho]) {
                            fila[fim++] = vizinho;
                            visitado[vizinho] = 1;
                        }
                    }
                }
            }

            fprintf(f, "Componente %d: %d vértices\n", componente, qtd);
            free(fila);
        }
    }

    free(visitado);
    fclose(f);
}

int diametro(Grafo *g) {
    if (g->n == 0) return 0;

    int *dist1 = bfs_distancias(g, 1);
    int v = 1, max_dist = -1;

    for (int i = 0; i < g->n; i++) {
        if (dist1[i] > max_dist) {
            max_dist = dist1[i];
            v = i + 1;
        }
    }
    free(dist1);

    if (max_dist == -1) {
        // Grafo desconexo (nenhum vértice acessível)
        return -1;
    }

    int *dist2 = bfs_distancias(g, v);
    max_dist = -1;
    for (int i = 0; i < g->n; i++) {
        if (dist2[i] > max_dist) {
            max_dist = dist2[i];
        }
    }
    free(dist2);

    return max_dist;
}

void libera_grafo(Grafo* g) {
    if (g->representacao == 0) {
        for (int i = 0; i < g->n; i++) free(g->matriz[i]);
        free(g->matriz);
    } else {
        for (int i = 0; i < g->n; i++) free(g->lista[i]);
        free(g->lista);
    }
    free(g->grau);
    free(g);

}

