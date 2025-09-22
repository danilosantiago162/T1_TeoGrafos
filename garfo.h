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

#endif