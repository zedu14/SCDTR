//
// Created by zedu on 05/12/19.
//


#ifndef CONTROLO_DISTRIBUIDO_CONTROLO_H
#define CONTROLO_DISTRIBUIDO_CONTROLO_H



#define N 3


float *multiply_vector(float *a, float *b);
float* sub_vector(float a[], float b[]);
float* multiply_c_vector(float a, float b[]);

class node {
public:
    int index;
    float d[N];
    float d_av[N];
    float y[N];
    float k[N];
    float n; //norma vetor k
    float m; //n-kii^2
    float c[N];
    float o;
    float L;

};

float norma(float *a);

void consensus_iterate(float (&d)[N], float& cost_in, node a, float rho);

int check_feasibility(node a, float d[]);
float cost(node a, float d[], float rho);

#endif //CONTROLO_DISTRIBUIDO_CONTROLO_H