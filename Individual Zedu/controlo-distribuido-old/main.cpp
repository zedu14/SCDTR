#include <iostream>
#include "controlo.h"
#include "node.h"

int main() {

    //auxiliaries to iterate
    int i=0;
    int j=0;

//Iluminâncias desejadas e perturbações externas;
float L1=150.0;
float O1=30.0;
float L2=80.0;
float O2=0.0;
float L3=100.0;
float O3=0.0;

//Custos
float c1=1.0;
float c2=1.0;
float c3=1.0;

//Parametros do solver
float rho=0.07;
int maxiter = 50;

//Parametros de calibracao
float k[N][N]={{2.0,1.0, 1.0}, {1.0,2.0, 1.0},{1.0,1.0,2.0}};
float c[N]={c1,c2,c3};
float L[N]={L1,L2,L3};
float O[N]={O1,O2,O3};

float costs[N];

//inicializar d's
float* d[3][3];
float* av[3];
float d_aux[3][3];

for(i=0;i<N;i++) {
    for (j = 0; j < N; j++) {
        d[i][j] = (float *) malloc(sizeof(float *) * maxiter);
        d_aux[i][j] = 0;
    }

    av[i]=(float*)malloc(sizeof(float*)*maxiter);
}



    //nodes creation
   node nodes[N];

    for(i=0;i<N;i++)
    {
        for(j=0;j<N;j++){
            nodes[i].k[j]=k[i][j];
        }
        nodes[i].n=norma(nodes[i].k);
        nodes[i].m=nodes[i].n-nodes[i].k[i];
        nodes[i].c[i]=c[i];
        nodes[i].o=O[i];
        nodes[i].L=L[i];

    }

    //solver distribuido
    //iteracao1
    for(i=0;i<N;i++) {
        for (j = 0; j < N; j++)
            d[i][j][0]=nodes[i].d[j];


    }

    for(i=0;i<N;i++) {
        for (j = 0; j < N; j++)
            av[i][0] = av[i][0]+d[j][i][0];
        av[i][0]=av[i][0]/N;
    }

    //restantes iteracoes
    for(i=1;i<maxiter;i++){
        consensus_iterate(d_aux[0], costs[0], nodes[0], rho);
        for(j=0;j<N;j++)
            nodes[0].d[j]=d_aux[0][j];

    }

    //debug
    for(i=0;i<N;i++)
            std::cout << nodes[i].n <<"\n";



    //libertacoes

    for(i=0;i<N;i++) {
        for (j = 0; j < N; j++)
            free(d[i][j]);

        free(av[i]);
    }
    return 0;
}
