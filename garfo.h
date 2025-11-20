#ifndef GRAFO_H
#define GRAFO_H

typedef struct No {
    int v;
    float peso; 
    struct No *prox;
} No;

typedef struct Grafo {
    int n;               // vértices
    int m;               // arestas
    int *grau;           // grau
    int direcionado;     // 0 = não dir., 1 = dir.
    int ponderado;       // 0 = sem peso, 1 = com peso
    int representacao;   // 0 = matriz, 1 = lista
    float **pesoVet;

    /* Matriz não ponderada */
    int **matriz;

    /* Matriz ponderada */
    float **matriz_p;

    int **adjVet; 

    /* Lista de adjacência (sempre encadeada) */
    No **lista;

} Grafo;

typedef struct {
    float *dist;   // distâncias finais
    int   *pai;    // pais da árvore de caminhos mínimos
    int   negativo; // 1 se ciclo negativo encontrado
} ResultadoBF;

typedef struct {
    int u, v;
    float w;
} Aresta;

// Criação e leitura
Grafo* cria_grafo(int n, int representacao, int direcionado, int ponderado);
void adiciona_aresta(Grafo *g, int u, int v, float peso);
Grafo* le_grafo(const char *filename, int representacao, int direcionado);
void libera_grafo(Grafo *g);

// Informações do grafo
void salva_informacoes(const char *filename, Grafo *g);
int grau_min(Grafo *g);
int grau_max(Grafo *g);
double grau_medio(Grafo *g);
double grau_mediana(Grafo *g);

// Buscas
void bfs(Grafo *g, int inicio, const char *saida);
int* bfs_distancias(Grafo *g, int inicio);
void dfs_iterativa(Grafo *g, int inicio, const char *saida);

// Distâncias
int distancia(Grafo *g, int u, int v);
int diametro(Grafo *g);

// Componentes conexas
void componentes_conexas(Grafo *g, const char *saida);

// Dijkstra (vetor e heap)
void dijkstra_vetor(Grafo *g, int origem, float *dist, int *pai);
void dijkstra_heap(Grafo *g, int origem, float *dist, int *pai);

Grafo* inverte_grafo(Grafo *g);

Aresta *gera_arestas_invertidas(Grafo *g, int *m_out);
ResultadoBF bellman_ford(Grafo *g, int destino);
void imprime_caminho_bf(int *pai, int destino, int origem);

// Auxiliares
void imprime_caminho(int *pai, int origem, int destino);
void estudo_caso_p(Grafo *g, int origem, int*alvos,int k);

#endif


