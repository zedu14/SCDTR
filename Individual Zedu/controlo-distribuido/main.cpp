#include <iostream>
#include "controlo.h"
#include "node.h"

int main() {

    //auxiliaries to iterate
    int i=0;
    int j=0;

//Iluminâncias desejadas e perturbações externas;
//float L1 = 150.0, O1 = 30.0, L2 = 80.0, O2 = 0.0;
// float L1 = 80.0, O1 = 50.0, L2 = 150.0, O2 = 50.0;
float L1 = 80.0, O1 = 50.0, L2 = 270.0, O2 = 50.0;
float L3=100.0;
float O3=0.0;

//Custos
//float c1=1.0, c2=1.0;
float c1=1.0, c2=3.0;
float c3=1.0;


//Parametros do solver
float rho=0.07;

//Parametros de calibracao
float k[N][N]={{2.0,1.0, 1.0}, {1.0,2.0, 1.0},{1.0,1.0,2.0}};
float c[N]={c1,c2,c3};
float L[N]={L1,L2,L3};
float O[N]={O1,O2,O3};

float costs[N]={0.0};

//inicializar d's
float d[N][N]={{0.0}};
float av[N]={0.0};
float d_aux[N]={0.0};
float d_av_aux[N]={0.0};
float d_final[N]={0.0};




    //nodes creation
   node nodes[N];

    for(i=0;i<N;i++)
    {
        nodes[i].index=i;
        for(j=0;j<N;j++){
            nodes[i].d[j]=0.0;
            nodes[i].d_av[j]=0.0;
            nodes[i].y[j]=0.0;
            nodes[i].k[j]=k[i][j];
        }
        nodes[i].n=norma_2(nodes[i].k, N);
        nodes[i].m=nodes[i].n-nodes[i].k[i]*nodes[i].k[i];
        nodes[i].c[i]=c[i];
        nodes[i].o=O[i];
        nodes[i].L=L[i];

    }

    //solver distribuido
    //iteracao1
    for(i=0;i<N;i++) {
        for (j = 0; j < N; j++)
            d[i][j]=nodes[i].d[j];
        
    }

    for(i=0;i<N;i++) {
        for (j = 0; j < N; j++)
            av[i] = av[i]+d[j][i];
        av[i]=av[i]/N;
    }

    //restantes iteracoes
    for(i=1;i<MaxIter;i++){

        for(j=0;j<N;j++) {
            costs[j]=consensus_iterate(d_aux, nodes[j], rho);
            equal_vector(nodes[j].d, d_aux, N);
        }


        //computar media

        calc_av(nodes,d_av_aux,N);
        for(j=0;j<N;j++) {
            equal_vector(nodes[j].d_av, d_av_aux, N);
        }
        //apagar d_av_aux
        for(j=0;j<N;j++) {
            d_av_aux[j]=0;
        }

        //computar lagrange updates
        update_lagrange(nodes,rho,N);

    }

    //resultados finais
    equal_vector(d_final,nodes[0].d_av,N);
    print_vector(d_final,N);



    return 0;
}
