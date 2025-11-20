#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <float.h>
#include <time.h>
#include "garfo.h"

/* ---------------------------
   Funções auxiliares internas
   --------------------------- */

/* verifica existência de aresta u->v de forma segura */
static int tem_aresta(Grafo *g, int u, int v) {
    if (!g) return 0;
    if (u < 0 || v < 0 || u >= g->n || v >= g->n) return 0;

    if (g->representacao == 0) {
        if (g->ponderado) {
            if (!g->matriz_p || !g->matriz_p[u]) return 0;
            return (g->matriz_p[u][v] != 0.0f);
        } else {
            if (!g->matriz || !g->matriz[u]) return 0;
            return (g->matriz[u][v] != 0);
        }
    } else if (g->representacao == 1) {
        if (!g->lista || !g->lista[u]) return 0;
        for (No *p = g->lista[u]; p; p = p->prox) {
            if (p->v == v) return (g->ponderado ? (p->peso != 0.0f) : 1);
        }
        return 0;
    } else { // adj vetor
        if (!g->adjVet || !g->adjVet[u]) return 0;
        for (int i = 0; i < g->grau[u]; i++) if (g->adjVet[u][i] == v) return 1;
        return 0;
    }
}

/* ---------------------------
   Criação e liberação
   --------------------------- */

Grafo* cria_grafo(int n, int representacao, int direcionado, int ponderado) {
    Grafo *g = (Grafo*) malloc(sizeof(Grafo));

    g->n = n;
    g->m = 0;
    g->representacao = representacao;
    g->direcionado = direcionado;
    g->ponderado = ponderado;

    g->grau = (int*) calloc(n, sizeof(int));

    // ================================
    // MATRIZ NÃO PONDERADA
    // ================================
    g->matriz = NULL;
    g->matriz_p = NULL;

    if (representacao == 0) {
        if (!ponderado) {
            g->matriz = (int**) malloc(n * sizeof(int*));
            for (int i = 0; i < n; i++) {
                g->matriz[i] = (int*) calloc(n, sizeof(int));
            }
        } 
        else {
            g->matriz_p = (float**) malloc(n * sizeof(float*));
            for (int i = 0; i < n; i++) {
                g->matriz_p[i] = (float*) calloc(n, sizeof(float));
            }
        }
    }

    // ================================
    // LISTA DE ADJACÊNCIA
    // ================================
    g->lista = NULL;

    if (representacao == 1) {
        g->lista = (No**) malloc(n * sizeof(No*));
        for (int i = 0; i < n; i++)
            g->lista[i] = NULL;
    }

    // ================================
    // VETOR DE ADJACÊNCIA + PESOS
    // ================================
    g->adjVet = NULL;
    g->pesoVet = NULL;

    if (representacao == 2) {
        g->adjVet = (int**) malloc(n * sizeof(int*));
        g->pesoVet = (float**) malloc(n * sizeof(float*));

        for (int i = 0; i < n; i++) {
            g->adjVet[i] = NULL;
            g->pesoVet[i] = NULL;
        }
    }

    return g;
}


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
    if (!copia) return 0.0; // ou tratar erro apropriadamente
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

void libera_grafo(Grafo *g) {
    if (!g) return;

    // -------------------------------------------------------
    // MATRIZ NÃO PONDERADA
    // -------------------------------------------------------
    if (g->representacao == 0 && g->matriz != NULL) {
        for (int i = 0; i < g->n; i++)
            free(g->matriz[i]);
        free(g->matriz);
        g->matriz = NULL;
    }

    // -------------------------------------------------------
    // MATRIZ PONDERADA
    // -------------------------------------------------------
    if (g->representacao == 0 && g->matriz_p != NULL) {
        for (int i = 0; i < g->n; i++)
            free(g->matriz_p[i]);
        free(g->matriz_p);
        g->matriz_p = NULL;
    }

    // -------------------------------------------------------
    // LISTA DE ADJACÊNCIA
    // -------------------------------------------------------
    if (g->representacao == 1 && g->lista != NULL) {
        for (int i = 0; i < g->n; i++) {
            No *p = g->lista[i];
            while (p != NULL) {
                No *tmp = p;
                p = p->prox;
                free(tmp);
            }
        }
        free(g->lista);
        g->lista = NULL;
    }

    // -------------------------------------------------------
    // VETOR DE ADJACÊNCIA + PESOS
    // -------------------------------------------------------
    if (g->representacao == 2 && g->adjVet != NULL) {
        for (int i = 0; i < g->n; i++) {
            free(g->adjVet[i]);
            free(g->pesoVet[i]);
        }
        free(g->adjVet);
        free(g->pesoVet);

        g->adjVet  = NULL;
        g->pesoVet = NULL;
    }

    // -------------------------------------------------------
    // GRAU
    // -------------------------------------------------------
    if (g->grau != NULL)
        free(g->grau);

    // -------------------------------------------------------
    // LIBERA A STRUCT
    // -------------------------------------------------------
    free(g);
}

/* ---------------------------
   Inserção unificada de aresta
   --------------------------- */

void adiciona_aresta(Grafo *g, int u, int v, float peso) {

    // -------------------------------------------------------------
    //   MATRIZ DE ADJACÊNCIA
    // -------------------------------------------------------------
    if (g->representacao == 0) {
        if (g->ponderado == 0) {
            g->matriz[u][v] = 1;
            if (!g->direcionado)
                g->matriz[v][u] = 1;
        } 
        else {
            g->matriz_p[u][v] = peso;
            if (!g->direcionado)
                g->matriz_p[v][u] = peso;
        }

        g->grau[u]++;
        if (!g->direcionado) g->grau[v]++;
        g->m++;
        return;
    }

    // -------------------------------------------------------------
    //   LISTA DE ADJACÊNCIA
    // -------------------------------------------------------------
    else if (g->representacao == 1) {

        No *novo = (No*) malloc(sizeof(No));
        novo->v = v;
        novo->peso = g->ponderado ? peso : 1.0;
        novo->prox = g->lista[u];
        g->lista[u] = novo;

        g->grau[u]++;

        if (!g->direcionado) {
            No *novo2 = (No*) malloc(sizeof(No));
            novo2->v = u;
            novo2->peso = g->ponderado ? peso : 1.0;
            novo2->prox = g->lista[v];
            g->lista[v] = novo2;
            g->grau[v]++;
        }

        g->m++;
        return;
    }

    // -------------------------------------------------------------
    //   VETOR DE ADJACÊNCIA + PESOS
    // -------------------------------------------------------------
    else if (g->representacao == 2) {

        int grau_u = g->grau[u] + 1;

        g->adjVet[u] = (int*)  realloc(g->adjVet[u],  grau_u * sizeof(int));
        g->pesoVet[u] = (float*) realloc(g->pesoVet[u], grau_u * sizeof(float));

        g->adjVet[u][grau_u - 1]  = v;
        g->pesoVet[u][grau_u - 1] = g->ponderado ? peso : 1.0;

        g->grau[u]++;

        if (!g->direcionado) {
            int grau_v = g->grau[v] + 1;

            g->adjVet[v] = (int*) realloc(g->adjVet[v], grau_v * sizeof(int));
            g->pesoVet[v] = (float*) realloc(g->pesoVet[v], grau_v * sizeof(float));

            g->adjVet[v][grau_v - 1]  = u;
            g->pesoVet[v][grau_v - 1] = g->ponderado ? peso : 1.0;

            g->grau[v]++;
        }

        g->m++;
        return;
    }
}


/* ---------------------------
   Leitura de arquivo (detecção automática de pesos)
   Formato:
     primeira linha: n
     linhas seguintes:
       u v            (não ponderado)
       u v w          (ponderado)
   --------------------------- */

Grafo* le_grafo(const char *filename, int representacao, int direcionado) {
    FILE *f = fopen(filename, "r");
    if (!f) {
        perror("Erro ao abrir arquivo");
        return NULL;
    }

    int n;
    fscanf(f, "%d", &n);

    int u, v;
    float w;

    // --------------------------------------------------------------------
    // DETECTAR AUTOMATICAMENTE SE O GRAFO É PONDERADO
    // --------------------------------------------------------------------
    int ponderado = 0;
    long pos = ftell(f);

    if (fscanf(f, "%d %d %f", &u, &v, &w) == 3)
        ponderado = 1;

    fseek(f, pos, SEEK_SET);  // volta para a posição inicial

    // --------------------------------------------------------------------
    // CRIA GRAFO
    // --------------------------------------------------------------------
    Grafo *g = cria_grafo(n, representacao, direcionado, ponderado);

    // --------------------------------------------------------------------
    // LEITURA DAS ARESTAS
    // --------------------------------------------------------------------
    if (!ponderado) {
        while (fscanf(f, "%d %d", &u, &v) == 2) {
            adiciona_aresta(g, u - 1, v - 1, 1.0);
        }
    }
    else {
        while (fscanf(f, "%d %d %f", &u, &v, &w) == 3) {
            adiciona_aresta(g, u - 1, v - 1, w);
        }
    }

    fclose(f);
    return g;
}


/* ---------------------------
   BFS: já suporta matriz/lista/vetor
   arquivo de saída: saida (nome) - se NULL imprime só no terminal
   --------------------------- */

void bfs(Grafo *g, int inicio, const char *saida) {
    printf("BFS REALMENTE ENTROU NA FUNÇÃO!\n");

    if (!g) return;
    inicio--; /* base 0 */
    if (inicio < 0 || inicio >= g->n) { fprintf(stderr,"bfs: inicio fora do intervalo\n"); return; }

    int *visitado = calloc(g->n, sizeof(int));
    int *pai = malloc(g->n * sizeof(int));
    int *nivel = malloc(g->n * sizeof(int));
    int *fila = malloc(g->n * sizeof(int));
    if (!visitado || !pai || !nivel || !fila) { perror("malloc"); exit(1); }

    for (int i = 0; i < g->n; i++) { pai[i] = -1; nivel[i] = -1; }

    int frente = 0, tras = 0;
    visitado[inicio] = 1;
    nivel[inicio] = 0;
    fila[tras++] = inicio;

    while (frente < tras) {
        int u = fila[frente++];

        if (g->representacao == 0) {
            /* matriz (ponderada ou não): checagem baseada em tem_aresta */
            for (int v = 0; v < g->n; v++) {
                if (tem_aresta(g, u, v) && !visitado[v]) {
                    visitado[v] = 1;
                    pai[v] = u;
                    nivel[v] = nivel[u] + 1;
                    fila[tras++] = v;
                }
            }
        } else if (g->representacao == 1) {
            /* lista encadeada */
            for (No *p = g->lista[u]; p; p = p->prox) {
                int v = p->v;
                if (!visitado[v]) {
                    visitado[v] = 1;
                    pai[v] = u;
                    nivel[v] = nivel[u] + 1;
                    fila[tras++] = v;
                }
            }
        } else {
            /* vetor de adjacência */
            for (int i = 0; i < g->grau[u]; i++) {
                int v = g->adjVet[u][i];
                if (!visitado[v]) {
                    visitado[v] = 1;
                    pai[v] = u;
                    nivel[v] = nivel[u] + 1;
                    fila[tras++] = v;
                }
            }
        }
    }

    FILE *f = NULL;
    if (saida) {
        f = fopen(saida, "w");
        if (!f) { perror("Erro ao abrir saida BFS"); f = NULL; }
    }

    if (f) fprintf(f, "BFS a partir do vertice %d\n\n", inicio + 1);
    printf("BFS a partir do vertice %d\n", inicio + 1);

    for (int i = 0; i < g->n; i++) {
        if (f) fprintf(f, "Vertice %d: pai = %d, nivel = %d\n", i+1, (pai[i]==-1? -1 : pai[i]+1), nivel[i]);
        printf("Vertice %d: pai = %d, nivel = %d\n", i+1, (pai[i]==-1? -1 : pai[i]+1), nivel[i]);
    }

    if (f) fclose(f);

    free(visitado); free(pai); free(nivel); free(fila);
}

/* ---------------------------
   DFS iterativa: suporta matriz/lista/vetor
   --------------------------- */

void dfs_iterativa(Grafo *g, int inicio, const char *saida) {
    if (!g) return;
    inicio--; if (inicio < 0 || inicio >= g->n) { fprintf(stderr,"dfs: inicio fora do intervalo\n"); return; }

    int *visitado = calloc(g->n, sizeof(int));
    int *pai = malloc(g->n * sizeof(int));
    int *nivel = malloc(g->n * sizeof(int));
    int *stack = malloc(g->n * sizeof(int));
    if (!visitado || !pai || !nivel || !stack) { perror("malloc"); exit(1); }

    for (int i = 0; i < g->n; i++) { pai[i] = -1; nivel[i] = -1; }

    FILE *f = NULL;
    if (saida) {
        f = fopen(saida, "w");
        if (!f) { perror("Erro ao abrir saida DFS"); f = NULL; }
    }
    if (f) fprintf(f, "DFS iterativa a partir do vertice %d\n", inicio + 1);
    printf("DFS iterativa a partir do vertice %d\n", inicio + 1);

    int top = -1;
    stack[++top] = inicio;
    nivel[inicio] = 0;

    while (top >= 0) {
        int u = stack[top--];
        if (visitado[u]) continue;
        visitado[u] = 1;

        if (f) fprintf(f, "Vertice %d: pai = %d, nivel = %d\n", u+1, (pai[u]==-1? -1 : pai[u]+1), nivel[u]);
        printf("Vertice %d: pai = %d, nivel = %d\n", u+1, (pai[u]==-1? -1 : pai[u]+1), nivel[u]);

        if (g->representacao == 0) {
            for (int v = 0; v < g->n; v++) {
                if (tem_aresta(g, u, v) && !visitado[v]) {
                    pai[v] = u;
                    nivel[v] = nivel[u] + 1;
                    stack[++top] = v;
                }
            }
        } else if (g->representacao == 1) {
            for (No *p = g->lista[u]; p; p = p->prox) {
                int v = p->v;
                if (!visitado[v]) {
                    pai[v] = u;
                    nivel[v] = nivel[u] + 1;
                    stack[++top] = v;
                }
            }
        } else {
            for (int i = 0; i < g->grau[u]; i++) {
                int v = g->adjVet[u][i];
                if (!visitado[v]) {
                    pai[v] = u;
                    nivel[v] = nivel[u] + 1;
                    stack[++top] = v;
                }
            }
        }
    }

    if (f) fprintf(f, "DFS concluída.\n");
    printf("DFS concluída.\n");
    if (f) fclose(f);

    free(visitado); free(pai); free(nivel); free(stack);
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
            for (No *p = g->lista[u]; p != NULL; p = p->prox) {
                int v = p->v;
                if (dist[v] == -1) {
                    dist[v] = dist[u] + 1;
                    fila[tras++] = v;
                }
            }
        }
    }

    free(fila);
    return dist; // vetor com as distâncias a partir de inicio
}

int distancia(Grafo *g, int u, int v) {
    int *d = bfs_distancias(g, u);
    if (!d) return -1;
    int res = d[v-1];
    free(d);
    return res;
}


int dfs_visit_cc(Grafo *g, int u, int *visitado) {
    visitado[u] = 1;
    int count = 1;

    if (g->representacao == 0) { // Matriz de adjacência
        for (int v = 0; v < g->n; v++) {
            if (g->matriz[u][v] && !visitado[v]) {
                count += dfs_visit_cc(g, v, visitado);
            }
        }
    } else { // Lista de adjacência
        for (No *p = g->lista[u]; p != NULL; p = p->prox) {
            int v = p->v;
            if (!visitado[v]) {
                count += dfs_visit_cc(g, v, visitado);
            }
        }
    }
    return count;
}

void componentes_conexas(Grafo *g, const char *saida) {
    FILE *f = fopen(saida, "w");
    if (!f) {
        perror("Erro ao salvar componentes conexas");
        return;
    }

    int *visitado = (int*) calloc(g->n, sizeof(int));
    int num_componentes = 0;
    int maior_componente = 0;
    int menor_componente = g->n + 1; // Inicializa com um valor maior que o possível

    fprintf(f, "Componentes Conexas:\n");
    printf("Componentes Conexas:\n");

    for (int i = 0; i < g->n; i++) {
        if (!visitado[i]) {
            num_componentes++;
            int tamanho_comp = dfs_visit_cc(g, i, visitado);

            fprintf(f, "Componente %d: %d vertices\n", num_componentes, tamanho_comp);
            printf("Componente %d: %d vertices\n", num_componentes, tamanho_comp);

            if (tamanho_comp > maior_componente) {
                maior_componente = tamanho_comp;
            }
            if (tamanho_comp < menor_componente) {
                menor_componente = tamanho_comp;
            }
        }
    }
}

int diametro(Grafo *g) {
    if (g->n == 0) return 0;

    int *visitado = (int*) calloc(g->n, sizeof(int));
    int maior_componente_tamanho = 0;
    int vertice_inicio = -1;

    // Encontra a maior componente conexa
    for (int i = 0; i < g->n; i++) {
        if (!visitado[i]) {
            int tamanho = dfs_visit_cc(g, i, visitado);
            if (tamanho > maior_componente_tamanho) {
                maior_componente_tamanho = tamanho;
                vertice_inicio = i + 1; // base 1 para bfs_distancias
            }
        }
    }

    if (vertice_inicio == -1) {
        free(visitado);
        return 0; // nenhum vértice acessível
    }

    free(visitado);

    // 1ª BFS: a partir de um vértice da maior componente
    int *dist1 = bfs_distancias(g, vertice_inicio);
    int v = vertice_inicio, max_dist = -1;

    for (int i = 0; i < g->n; i++) {
        if (dist1[i] > max_dist) {
            max_dist = dist1[i];
            v = i + 1; // vértice mais distante (base 1)
        }
    }
    free(dist1);

    if (max_dist == -1) return -1; // grafo desconexo

    // 2ª BFS: a partir do vértice mais distante encontrado
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

// ---------- Dijkstra (versão vetor) ----------
void dijkstra_vetor(Grafo *g, int origem, float *dist, int *pai) {
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

        for (No *p = g->lista[u]; p; p = p->prox) {
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
void dijkstra_heap(Grafo *g, int origem, float *dist, int *pai) {
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

        for (No *p = g->lista[u]; p; p = p->prox) {
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

Grafo* inverte_grafo(Grafo *g) {
    Grafo *inv = cria_grafo(g->n, g->representacao, 1, g->ponderado);
    // ============================================================
    // MATRIZ DE ADJACÊNCIA (ponderada ou não)
    // ============================================================
    if (g->representacao == 0) {
        for (int u = 0; u < g->n; u++) {
            for (int v = 0; v < g->n; v++) {

                if (!g->ponderado) {
                    if (g->matriz[u][v] != 0)
                        adiciona_aresta(inv, v, u, 1.0);
                }
                else {
                    if (g->matriz_p[u][v] != 0)
                        adiciona_aresta(inv, v, u, g->matriz_p[u][v]);
                }
            }
        }
        return inv;
    }
    // ============================================================
    // LISTA DE ADJACÊNCIA
    // ============================================================
    if (g->representacao == 1) {

        for (int u = 0; u < g->n; u++) {
            for (No *p = g->lista[u]; p != NULL; p = p->prox) {
                int v = p->v;
                float w = g->ponderado ? p->peso : 1.0;
                adiciona_aresta(inv, v, u, w);
            }
        }
        return inv;
    }
    // ============================================================
    // VETOR DE ADJACÊNCIA + PESOS
    // ============================================================
    if (g->representacao == 2) {
        for (int u = 0; u < g->n; u++) {

            for (int i = 0; i < g->grau[u]; i++) {
                int v = g->adjVet[u][i];
                float w = g->ponderado ? g->pesoVet[u][i] : 1.0;

                adiciona_aresta(inv, v, u, w);
            }
        }
        return inv;
    }
    return inv;
}

// ---------- Funções auxiliares ----------
void imprime_caminho(int *pai, int origem, int destino) {
    if (origem == destino)
        printf("%d", origem + 1);
    else if (pai[destino] == -1)
        printf("Sem caminho");
    else {
        imprime_caminho(pai, origem, pai[destino]);
        printf(" -> %d", destino + 1);
    }
}

// ==========================================================
//   BELLMAN–FORD
// ==========================================================

// ---------------------------------------------------------------
// Gera a lista de arestas INVERTIDAS, igual ao Dijkstra-reverso
// (se no arquivo existe u -> v, aqui teremos v -> u)
// ---------------------------------------------------------------
Aresta *gera_arestas_invertidas(Grafo *g, int *m_out) {
    int cap = 128;
    int count = 0;
    Aresta *E = malloc(cap * sizeof(Aresta));

    if (!E) { perror("malloc"); exit(1); }

    if (g->representacao == 1) {   // LISTA
        for (int u = 0; u < g->n; u++) {
            for (No *p = g->lista[u]; p != NULL; p = p->prox) {
                int v = p->v;
                float w = g->ponderado ? p->peso : 1.0f;

                if (count >= cap) {
                    cap *= 2;
                    E = realloc(E, cap * sizeof(Aresta));
                }
                // invertida: v -> u
                E[count++] = (Aresta){ v, u, w };
            }
        }
    }

    else if (g->representacao == 0) { // MATRIZ
        if (g->ponderado) {
            for (int u = 0; u < g->n; u++)
                for (int v = 0; v < g->n; v++)
                    if (g->matriz_p[u][v] != 0) {
                        if (count >= cap) { cap *= 2; E = realloc(E, cap * sizeof(Aresta)); }
                        E[count++] = (Aresta){ v, u, g->matriz_p[u][v] };
                    }
        } else {
            for (int u = 0; u < g->n; u++)
                for (int v = 0; v < g->n; v++)
                    if (g->matriz[u][v] != 0) {
                        if (count >= cap) { cap *= 2; E = realloc(E, cap * sizeof(Aresta)); }
                        E[count++] = (Aresta){ v, u, 1.0f };
                    }
        }
    }

    else { // VETOR DE ADJACÊNCIA
        for (int u = 0; u < g->n; u++) {
            for (int k = 0; k < g->grau[u]; k++) {
                int v = g->adjVet[u][k];
                float w = g->ponderado ? g->pesoVet[u][k] : 1.0f;

                if (count >= cap) { cap *= 2; E = realloc(E, cap * sizeof(Aresta)); }

                // invertida v -> u
                E[count++] = (Aresta){ v, u, w };
            }
        }
    }

    *m_out = count;
    return E;
}

// ---------------------------------------------------------------
// B E L L M A N – F O R D  usando arestas invertidas
// Encontra distâncias DE TODOS para o vértice t.
// ---------------------------------------------------------------
#include <math.h>
ResultadoBF bellman_ford(Grafo *g, int t_orig) {
    ResultadoBF R = {0};

    int n = g->n;
    int t = t_orig - 1; // base 0

    float *dist = malloc(n * sizeof(float));
    int   *pai  = malloc(n * sizeof(int));
    if (!dist || !pai) { perror("malloc"); exit(1); }

    // inicialização
    for (int i = 0; i < n; i++) {
        dist[i] = INFINITY;
        pai[i] = -1;
    }
    dist[t] = 0.0f;

    // gera arestas invertidas
    int m = 0;
    Aresta *E = gera_arestas_invertidas(g, &m);

    // relaxações (n-1 vezes)
    for (int i = 0; i < n - 1; i++) {
        int mudou = 0;

        for (int k = 0; k < m; k++) {
            int u = E[k].u;
            int v = E[k].v;
            float w = E[k].w;

            if (dist[u] != INFINITY && dist[u] + w < dist[v]) {
                dist[v] = dist[u] + w;
                pai[v] = u;
                mudou = 1;
            }
        }

        if (!mudou) break; // otimização 1: early stopping
    }

    // detecção de ciclo negativo
    int negativo = 0;
    for (int k = 0; k < m; k++) {
        int u = E[k].u;
        int v = E[k].v;
        float w = E[k].w;

        if (dist[u] != INFINITY && dist[u] + w < dist[v]) {
            negativo = 1;
            break;
        }
    }

    free(E);

    R.dist = dist;
    R.pai = pai;
    R.negativo = negativo;
    return R;
}

void imprime_caminho_bf(int *pai, int destino, int atual) {
    if (atual == destino) {
        printf("%d", destino + 1);
        return;
    }
    if (pai[atual] == -1) {
        printf("sem caminho");
        return;
    }
    imprime_caminho_bf(pai, destino, pai[atual]);
    printf(" -> %d", atual + 1);
}
// ---------- Estudo de caso (até item 2) ----------
void estudo_caso_p(Grafo *g, int origem, int *alvos, int k) {
    float *distV = malloc(g->n * sizeof(float));
    int *paiV = malloc(g->n * sizeof(int));
    float *distH = malloc(g->n * sizeof(float));
    int *paiH = malloc(g->n * sizeof(int));

    printf("\n=== Dijkstra Vetor ===\n");
    dijkstra_vetor(g, origem - 1, distV, paiV);
    for (int i = 0; i < 5; i++) {
        int t = alvos[i] - 1;
        printf("Dist(%d -> %d) = %.3f | Caminho: ", origem, alvos[i], distV[t]);
        imprime_caminho(paiV, origem - 1, t);
        printf("\n");
    }

    printf("\n=== Dijkstra Heap ===\n");
    dijkstra_heap(g, origem - 1, distH, paiH);
    for (int i = 0; i < 5; i++) {
        int t = alvos[i] - 1;
        printf("Dist(%d -> %d) = %.3f | Caminho: ", origem, alvos[i], distH[t]);
        imprime_caminho(paiH, origem - 1, t);
        printf("\n");
    }

    // Comparação de tempo médio
    srand(42);
    clock_t t0 = clock();
    for (int i = 0; i < k; i++) {
        int s = rand() % g->n;
        dijkstra_vetor(g, s, distV, paiV);
    }
    clock_t t1 = clock();
    for (int i = 0; i < k; i++) {
        int s = rand() % g->n;
        dijkstra_heap(g, s, distH, paiH);
    }
    clock_t t2 = clock();

    double tv = (double)(t1 - t0) / CLOCKS_PER_SEC / k;
    double th = (double)(t2 - t1) / CLOCKS_PER_SEC / k;
    printf("\nTempo médio (k=%d): Vetor = %.6fs | Heap = %.6fs\n", k, tv, th);

    free(distV); free(paiV);
    free(distH); free(paiH);
}


