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
GrafoP* cria_grafo_p(int n) {
    GrafoP *g = (GrafoP*) malloc(sizeof(GrafoP));
    g->n = n;
    g->m = 0;
    g->adj = (NoP**) calloc(n, sizeof(NoP*));
    return g;
}

void adiciona_aresta_p(GrafoP *g, int u, int v, float peso) {
    NoP *novo = (NoP*) malloc(sizeof(NoP));
    novo->v = v;
    novo->peso = peso;
    novo->prox = g->adj[u];
    g->adj[u] = novo;

    NoP *novo2 = (NoP*) malloc(sizeof(NoP)); // não direcionado
    novo2->v = u;
    novo2->peso = peso;
    novo2->prox = g->adj[v];
    g->adj[v] = novo2;

    g->m++;
}

GrafoP* le_grafo_pesos(const char *filename) {
    FILE *f = fopen(filename, "r");
    if (!f) { perror("Erro ao abrir arquivo"); exit(1); }

    int n;
    fscanf(f, "%d", &n); // lê número de vértices
    GrafoP *g = cria_grafo_p(n);

    int u, v;
    float w;
    while (fscanf(f, "%d %d %f", &u, &v, &w) == 3) {
        adiciona_aresta_p(g, u-1, v-1, w);
    }

    fclose(f);
    return g;
}

void libera_grafo_p(GrafoP *g) {
    for (int i = 0; i < g->n; i++) {
        NoP *atual = g->adj[i];
        while (atual) {
            NoP *tmp = atual;
            atual = atual->prox;
            free(tmp);
        }
    }
    free(g->adj);
    free(g);
}

// ---------- Dijkstra (versão vetor) ----------
void dijkstra_vetor(GrafoP *g, int origem, float *dist, int *pai) {
    int n = g->n;
    int *visitado = calloc(n, sizeof(int));
    for (int i = 0; i < n; i++) {
        dist[i] = FLT_MAX;
        pai[i] = -1;
    }
    dist[origem] = 0;

    for (int c = 0; c < n; c++) {
        int u = -1;
        float min = FLT_MAX;
        for (int i = 0; i < n; i++)
            if (!visitado[i] && dist[i] < min) {
                min = dist[i];
                u = i;
            }

        if (u == -1) break;
        visitado[u] = 1;

        for (NoP *p = g->adj[u]; p; p = p->prox) {
            int v = p->v;
            float w = p->peso;
            if (dist[u] + w < dist[v]) {
                dist[v] = dist[u] + w;
                pai[v] = u;
            }
        }
    }

    free(visitado);
}

// ---------- Estruturas auxiliares para heap ----------
typedef struct {
    int v;
    float dist;
} Par;

typedef struct {
    Par *vet;
    int tamanho;
} HeapMin;

void troca_p(Par *a, Par *b) {
    Par tmp = *a; *a = *b; *b = tmp;
}

void sobe_p(HeapMin *h, int i) {
    while (i > 0) {
        int p = (i - 1) / 2;
        if (h->vet[i].dist >= h->vet[p].dist) break;
        troca_p(&h->vet[i], &h->vet[p]);
        i = p;
    }
}

void desce_p(HeapMin *h, int i) {
    int esq, dir, menor;
    while (1) {
        esq = 2*i + 1;
        dir = 2*i + 2;
        menor = i;
        if (esq < h->tamanho && h->vet[esq].dist < h->vet[menor].dist) menor = esq;
        if (dir < h->tamanho && h->vet[dir].dist < h->vet[menor].dist) menor = dir;
        if (menor == i) break;
        troca_p(&h->vet[i], &h->vet[menor]);
        i = menor;
    }
}

void inserir_heap_p(HeapMin *h, int v, float dist) {
    h->vet[h->tamanho].v = v;
    h->vet[h->tamanho].dist = dist;
    sobe_p(h, h->tamanho++);
}

Par extrair_min_p(HeapMin *h) {
    Par raiz = h->vet[0];
    h->vet[0] = h->vet[--h->tamanho];
    desce_p(h, 0);
    return raiz;
}

// ---------- Dijkstra (versão heap) ----------
void dijkstra_heap(GrafoP *g, int origem, float *dist, int *pai) {
    int n = g->n;
    for (int i = 0; i < n; i++) {
        dist[i] = FLT_MAX;
        pai[i] = -1;
    }
    dist[origem] = 0;

    HeapMin h;
    h.vet = malloc(n * n * sizeof(Par));
    h.tamanho = 0;

    inserir_heap_p(&h, origem, 0.0);

    while (h.tamanho > 0) {
        Par atual = extrair_min_p(&h);
        int u = atual.v;
        float d = atual.dist;
        if (d > dist[u]) continue;

        for (NoP *p = g->adj[u]; p; p = p->prox) {
            int v = p->v;
            float w = p->peso;
            if (dist[u] + w < dist[v]) {
                dist[v] = dist[u] + w;
                pai[v] = u;
                inserir_heap_p(&h, v, dist[v]);
            }
        }
    }

    free(h.vet);
}

// ---------- Funções auxiliares ----------
void imprime_caminho_p(int *pai, int origem, int destino) {
    if (origem == destino)
        printf("%d", origem + 1);
    else if (pai[destino] == -1)
        printf("Sem caminho");
    else {
        imprime_caminho_p(pai, origem, pai[destino]);
        printf(" -> %d", destino + 1);
    }
}
NomeMap* le_mapa_nomes(const char *filename) {
    FILE *f = fopen(filename, "r");
    if (!f) {
        perror("Erro ao abrir arquivo de nomes");
        exit(1);
    }

    NomeMap *map = malloc(sizeof(NomeMap));
    map->nomes = NULL;
    map->n = 0;

    char linha[512];
    while (fgets(linha, sizeof(linha), f)) {
        // Remove \r e \n
        linha[strcspn(linha, "\r\n")] = '\0';

        // Divide a linha em duas partes: id e nome
        char *token = strtok(linha, ",");
        if (!token) continue;

        int id = atoi(token);
        char *nome = strtok(NULL, ",");
        if (!nome) continue;

        // Ajusta o tamanho do vetor
        if (id > map->n) {
            map->nomes = realloc(map->nomes, id * sizeof(char*));
            for (int i = map->n; i < id; i++)
                map->nomes[i] = NULL;
            map->n = id;
        }

        map->nomes[id - 1] = strdup(nome);
    }

    fclose(f);
    return map;
}

// ---------- Busca índice pelo nome ----------
int indice_por_nome(NomeMap *map, const char *nome) {
    for (int i = 0; i < map->n; i++)
        if (map->nomes[i] && strcmp(map->nomes[i], nome) == 0)
            return i;
    return -1;
}

// ---------- Liberação do mapa ----------
void libera_mapa_nomes(NomeMap *map) {
    for (int i = 0; i < map->n; i++)
        free(map->nomes[i]);
    free(map->nomes);
    free(map);
}

// ---------- Estudo de caso com nomes ----------
void estudo_caso_pesquisadores(const char *arquivo_grafo, const char *arquivo_nomes,
                               const char *origem_nome, const char **destinos, int qtd_destinos)
{
    GrafoP *g = le_grafo_pesos(arquivo_grafo);
    NomeMap *map = le_mapa_nomes(arquivo_nomes);

    int origem = indice_por_nome(map, origem_nome);
    if (origem == -1) {
        printf("Pesquisador '%s' não encontrado!\n", origem_nome);
        libera_mapa_nomes(map);
        libera_grafo_p(g);
        return;
    }

    float *dist = malloc(g->n * sizeof(float));
    int *pai = malloc(g->n * sizeof(int));

    printf("\n=== Dijkstra (heap) a partir de '%s' ===\n", origem_nome);
    dijkstra_heap(g, origem, dist, pai);

    for (int i = 0; i < qtd_destinos; i++) {
        int destino = indice_por_nome(map, destinos[i]);
        if (destino == -1) {
            printf("Pesquisador '%s' não encontrado!\n", destinos[i]);
            continue;
        }
        printf("\nDistância '%s' -> '%s' = %.6f\n", origem_nome, destinos[i], dist[destino]);
        printf("Caminho: ");
        imprime_caminho_p(pai, origem, destino);
        printf("\n");
    }

    free(dist);
    free(pai);
    libera_mapa_nomes(map);
    libera_grafo_p(g);
}

