//
// Created by zedu on 05/12/19.
//


#ifndef CONTROLO_DISTRIBUIDO_CONTROLO_H
#define CONTROLO_DISTRIBUIDO_CONTROLO_H



#define N 3
#define MaxIter 50

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

void sub_vector(float a[], float b[],float c[],int l);
void add_vector(float a[], float b[],float c[],int l);
void multiply_c_vector(float a, float b[],float c[],int l);
void equal_vector(float dest[],float from[],int l);
float multiply_vector_to_float(float a[],float b[],int l);


float norma_2(float *a,int l);

float consensus_iterate(float d[], node a, float rho);

int check_feasibility(node a, float d[]);
float cost(node a, float d[], float rho);
void calc_av(float ds[][],float d_av_aux[],int l);
void update_lagrange(node a,float rho);
void print_vector(float d[], int l);
void update_av(int i, float av_arduino[], float d[],int i);

#endif //CONTROLO_DISTRIBUIDO_CONTROLO_H
