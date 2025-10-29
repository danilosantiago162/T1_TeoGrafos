#ifndef GRAFO_H
#define GRAFO_H

//teste

typedef struct Grafo {
    int n;              // número de vértices
    int m;              // número de arestas
    int **matriz;       // matriz de adjacência
    int **lista;        // lista de adjacência
    int *grau;          // grau de cada vértice
    int representacao;  // 0 = matriz, 1 = lista
} Grafo;

// Criação e leitura
Grafo* cria_grafo(int n, int representacao);
Grafo* le_grafo_arquivo(const char *filename, int representacao);
void libera_grafo(Grafo* g);

// Informações do grafo
void salva_informacoes(const char *filename, Grafo *g);
int grau_min(Grafo *g);
int grau_max(Grafo *g);
double grau_medio(Grafo *g);
double grau_mediana(Grafo *g);

// Buscas
void bfs(Grafo *g, int inicio, const char *saida);
void dfs(Grafo *g, int inicio, const char *saida);

// Distâncias
int distancia(Grafo *g, int u, int v);
int diametro(Grafo *g);

// Componentes conexas
void componentes_conexas(Grafo *g, const char *saida);

GrafoP* cria_grafo_p(int n);
void adiciona_aresta_p(GrafoP *g, int u, int v, float peso);
GrafoP* le_grafo_pesos(const char *filename);
void libera_grafo_p(GrafoP *g);

// Dijkstra (vetor e heap)
void dijkstra_vetor(GrafoP *g, int origem, float *dist, int *pai);
void dijkstra_heap(GrafoP *g, int origem, float *dist, int *pai);

// Auxiliares
void imprime_caminho_p(int *pai, int origem, int destino);
void estudo_caso_p(GrafoP *g, int origem, int *alvos, int k);

void estudo_caso_p(GrafoP *g, int origem, int *alvos, int k);

typedef struct {
    char **nomes;
    int n;
} NomeMap;

NomeMap* le_mapa_nomes(const char *filename);
int indice_por_nome(NomeMap *map, const char *nome);
void libera_mapa_nomes(NomeMap *map);

void estudo_caso_pesquisadores(const char *arquivo_grafo, const char *arquivo_nomes,
                               const char *origem_nome, const char **destinos, int qtd_destinos);


#endif

