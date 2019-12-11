//
// Created by zedu on 05/12/19.
//
#include "math.h"
#include "controlo.h"
#include "coms.h"

#define ENVIAR_DAV_OUTROS 10
#define PEDIR_DAV_OUTROS 11
#define ENVIAR_D_PARA_AV 12
#define PEDIR_D_PARA_AV 13



void sub_vector(float a[], float b[], float c[],int l) {
    int i = 0;

    for (i = 0; i < l; i++)
        c[i] = a[i]-b[i];

}

void add_vector(float a[], float b[], float c[],int l) {
    int i = 0;

    for (i = 0; i < l; i++)
        c[i] = a[i]+b[i];

}

void multiply_c_vector(float a, float b[], float c[],int l) {
    int i = 0;

    for (i = 0; i < l; i++)
        c[i] = a*b[i];
}

void equal_vector(float dest[],float from[],int l){
    int i=0;

    for(i=0;i<l;i++)
        dest[i]=from[i];
}

float multiply_vector_to_float(float a[],float b[],int l){
    float res=0.0;
    int i=0;
    for(i=0;i<l;i++)
        res+=a[i]*b[i];

    return res;
}

float norma_2( float a[],int l){
        int i=0;
        float n=0.0;

        for(i=0; i<l;i++){
            n+=a[i]*a[i];
        }
        return n;
}

//passa vetor d e custo por referencia
float consensus_iterate(float d[], node a, float rho) {


    int i = 0;
    float d_best[N]={0.0};
    for (i = 0; i < N; i++)
        d_best[i] = -1.0;


    int sol_unconstrained=1;
    int sol_boundary_linear=1;
    int sol_boundary_0=1;
    int sol_boundary_100=1;
    int sol_linear_0=1;
    int sol_linear_100=1;

    float cost_best = 1000000.0;
    float cost_unconstrained=1000000.0;
    float cost_boundary_linear=1000000.0;
    float cost_boundary_0=1000000.0;
    float cost_boundary_100=1000000.0;
    float cost_linear_0=1000000.0;
    float cost_linear_100=1000000.0;


    float z[N]={0.0};

    float d_u[N]={0.0};
    float d_bl[N]={0.0};
    float d_b0[N]={0.0};
    float d_b1[N]={0.0};
    float d_l0[N]={0.0};
    float d_l1[N]={0.0};

    float aux1[N]={0.0};
    float aux2[N]={0.0};

    multiply_c_vector(rho,a.d_av, aux1,N);
    sub_vector(aux1,a.y, aux2,N);
    sub_vector(aux2,a.c,z,N);


    multiply_c_vector(1.0/rho,z,d_u,N);
    sol_unconstrained = check_feasibility(a, d_u);
    if (sol_unconstrained == 1) {
        cost_unconstrained = cost(a, d_u, rho);
        if (cost_unconstrained < cost_best) {
            equal_vector(d_best,d_u,N);
            cost_best = cost_unconstrained;
        }
    }


    multiply_c_vector((a.o - a.L + 1 / rho * multiply_vector_to_float(z, a.k,N))/ a.n, a.k,aux1,N);
    multiply_c_vector(1.0 / rho,z,aux2,N);
    sub_vector(aux2,aux1,d_bl,N);

    sol_boundary_linear = check_feasibility(a, d_bl);

    if (sol_boundary_linear == 1) {
        cost_boundary_linear = cost(a, d_bl, rho);
        if (cost_boundary_linear < cost_best) {
            equal_vector(d_best,d_bl,N);
            cost_best = cost_boundary_linear;
        }
    }

    multiply_c_vector(1.0/rho,z,d_b0,N);
    d_b0[a.index] = 0.0;

    sol_boundary_0 = check_feasibility(a, d_b0);

    if (sol_boundary_0 == 1) {
        cost_boundary_0 = cost(a, d_b0, rho);
        if (cost_boundary_0 < cost_best) {
            equal_vector(d_best,d_b0,N);
            cost_best = cost_boundary_0;
        }
    }

    multiply_c_vector(1.0/rho,z,d_b1,N);
    d_b1[a.index] = 100.0;

    sol_boundary_100 = check_feasibility(a, d_b1);

    if (sol_boundary_100==1){
        cost_boundary_100 = cost(a, d_b1, rho);
        if (cost_boundary_100 < cost_best){
            equal_vector(d_best,d_b1,N);
            cost_best = cost_boundary_100;
        }
    }


    multiply_c_vector(1.0/rho,z,aux1,N);
    multiply_c_vector(1.0/a.m*(a.o-a.L),a.k,aux2,N);
    sub_vector(aux1,aux2,d_l0,N);
    multiply_c_vector((1.0/rho*1.0/a.m)*(a.k[a.index]*z[a.index]-multiply_vector_to_float(z,a.k,N)),a.k,aux1,N);
    add_vector(d_l0,aux1,d_l0,N);

    d_l0[a.index] = 0.0;

    sol_linear_0 = check_feasibility(a, d_l0);

    if (sol_linear_0==1) {
        cost_linear_0 = cost(a, d_l0, rho);
        if (cost_linear_0 < cost_best) {
            equal_vector(d_best,d_l0,N);
            cost_best = cost_linear_0;

        }

    }

    multiply_c_vector(1.0/rho,z,aux1,N);
    multiply_c_vector(1.0/a.m*(a.o-a.L+100*a.k[a.index]),a.k,aux2,N);
    sub_vector(aux1,aux2,d_l1,N);
    multiply_c_vector((1.0/rho*1.0/a.m)*(a.k[a.index]*z[a.index]-multiply_vector_to_float(z,a.k,N)),a.k,aux1,N);
    add_vector(d_l1,aux1,d_l1,N);

    d_l1[a.index] = 100.0;

    sol_linear_100 = check_feasibility(a, d_l1);

    if (sol_linear_100==1) {
        cost_linear_100 = cost(a, d_l1, rho);
        if (cost_linear_100 < cost_best) {
            equal_vector(d_best,d_l1,N);
            cost_best = cost_linear_100;
        }
    }
    equal_vector(d,d_best,N);

    return cost_best;
}

int check_feasibility(node a, float d[]){
    float tol = 0.001;

    if (d[a.index] < 0.0-tol) return 0;
    if (d[a.index] > 100.0+tol) return 0;
    if (multiply_vector_to_float(d,a.k,N) < a.L-a.o-tol) return 0;

    return 1;

}

float cost(node a, float d[], float rho){
    float aux1[N]={0.0};
    float norm_aux;

    sub_vector(d,a.d_av,aux1,N);
    norm_aux=norma_2(aux1,N);

    return  multiply_vector_to_float(a.c,d,N)+multiply_vector_to_float(a.y,aux1,N)+rho/2.0*norm_aux;

}


void update_lagrange(node a,float rho){
  int i=0;
  float aux1[N]={0.0};
  
  sub_vector(a.d,a.d_av,aux1,N);
  multiply_c_vector(rho,aux1,aux1,N);
  add_vector(a.y,aux1,a.y,N);
}



void update_av(int i, float av[], float d[]){
  int j=0;
  float dji=0.0;
  float d_avji=0.0;
  
  for(j=0;j<N;j++){
   av[j]=0;
   if(i!=j){
    //pedir o que  que os outros acham que eu sou
     dji=request(PEDIR_D_PARA_AV, i, j); 
     av[j]+=dji;
     
     //envio tambem o que acho que eles sao porque os outros vao precisar
     send_data(ENVIAR_D_PARA_AV, d[j], i, j);
   }
   else{
     //sou eu mesmo,nao tenho que pedir
     av[j]+=d[i];
   }
   
 }

//completar vetor

 for(j=0;j<N;j++){
   if(i!=j){
     //av vem dos outros
     d_avji=request(PEDIR_DAV_OUTROS, i, j);
     av[j]=d_avji ; 
   }
   else{
     av[i]=av[i]/N;
     send_data(ENVIAR_DAV_OUTROS, d[j], i, j);
   }
 }
}


float controlo_distribuido(float L, float O, float c, float k[], int i){
  int iter=0;
  float av_arduino[N]={0.0};
  float custo=0.0;
  float d[N]={0.0};
  float rho=0.07;
  int j =0;
  
  node arduino;
  arduino.index=i;
  for(j=0;j<N;j++){
    arduino.d[j]=0.0;
    arduino.d_av[j]=0.0;
    arduino.y[j]=0.0;
    arduino.k[j]=0.0;
  }
  
  equal_vector(arduino.k,k,N); 
  arduino.n=norma_2(arduino.k, N);
  arduino.m=arduino.n-arduino.k[i]*arduino.k[i];
  arduino.c[i]=c;
  arduino.o=O;
  arduino.L=L;
  
  for(iter=1;iter<MaxIter;iter++){
    custo=consensus_iterate(d, arduino, rho);
 
     //computar media
    update_av(i, av_arduino,d);

    //atualizar lagrange
    update_lagrange(arduino,rho);
  }
  
  return av_arduino[i];
}
