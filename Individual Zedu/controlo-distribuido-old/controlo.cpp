//
// Created by zedu on 05/12/19.
//
#include <iostream>
#include "math.h"
#include "controlo.h"

float* multiply_vector(float a[], float b[]) {
    int i = 0;

    int l1 = sizeof(a);
    int l2 = sizeof(b);

    float* c = (float*)malloc(sizeof(float) * l1);

    //vetores tamanhos diferentes
    if (l1 != l2) {
        for (i = 0; i < l1; i++)
            c[i] = 0;
    } else {
        for (i = 0; i < l1; i++)
            c[i] = a[i]*b[i];
    }

    return c;
}

float* sub_vector(float a[], float b[]) {
    int i = 0;

    int l1 = sizeof(a)/sizeof(*a);
    int l2 = sizeof(b)/sizeof(*b);

    float* c = (float*)malloc(sizeof(float) * l1);

    //vetores tamanhos diferentes
    if (l1 != l2) {
        for (i = 0; i < l1; i++)
            c[i] = 0;
    } else {
        for (i = 0; i < l1; i++)
            c[i] = a[i]-b[i];
    }

    return c;
}

float* multiply_c_vector(float a, float b[]) {
    int i = 0;

    int l1 = sizeof(b)/sizeof(*b);

    float* c = (float*)malloc(sizeof(float) * l1);

    //vetores tamanhos diferentes

    for (i = 0; i < l1; i++)
        c[i] = a*b[i];

    return c;
}

float norma( float a[]){
        int i=0;
        float n=0;
        int l=0;
        l=sizeof(a);

        for(i=0; i<l;i++){
            n=n+a[i]*a[i];
        }
        n=sqrt(n);
        return n;
}

//passa vetor d e custo por referencia
void consensus_iterate(float (&d)[N], float& cost_in, node a, float rho) {


    int i = 0;
    float d_best[N];
    for (i = 0; i < N; i++)
        d_best[i] = -1;

    float cost_best = 1000000;
    float sol_unconstrained = 1;
    float sol_boundary_linear = 1;
    float sol_boundary_0 = 1;
    float sol_boundary_100 = 1;
    float sol_linear_0 = 1;
    float sol_linear_100 = 1;
    float cost_unconstrained = 0;

    float z[N]={0};

    float d_u[N];
    float d_bl[N];
    float d_b0[N];
    float d_b1[N];
    float d_bl0[N];
    float d_bl1[N];

    float aux1[3]={0};
    float aux2[3]={0};

    aux1=multiply_c_vector(rho,a.d_av);
    aux2=sub_vector(aux1,a.y);
    z = sub_vector(aux2,a.c);

    d_u = 1 / rho * z;
    sol_unconstrained = check_feasibility(a, d_u);
    if (sol_unconstrained == 0) {
        cost_unconstrained = cost(a, d_u, rho);
        if (cost_unconstrained < cost_best) {
            d_best = d_u;
            cost_best = cost_unconstrained;
        }
    }

    d_bl = 1 / rho * z - a.k / a.n * (a.o - a.L + 1 / rho * multiply_vector(z, a.k));
    sol_boundary_linear = check_feasibility(a, d_bl);

    if (sol_boundary_linear == 0) {
        cost_boundary_linear = cost(a, d_bl, rho);
        if (cost_boundary_linear < cost_best) {
            d_best = d_bl;
            cost_best = cost_boundary_linear;
        }
    }

    d_b0 = (1 / rho) * z;
    d_b0[a.index] = 0;

    sol_boundary_0 = check_feasibility(a, d_b0);

    if (sol_boundary_0 == 0) {
        cost_boundary_0 = cost(a, d_b0, rho);
        if (cost_boundary_0 < cost_best) {
            d_best = d_b0;
            cost_best = cost_boundary_0;
        }
    }

    d_b1 = (1 / rho) * z;
    d_b1[a.index] = 100;

    sol_boundary_100 = check_feasibility(a, d_b1);

    if (sol_boundary_100==0){
        cost_boundary_100 = cost(a, d_b1, rho);
        if (cost_boundary_100 < cost_best){
            d_best = d_b1;
            cost_best = cost_boundary_100;
        }
    }

    d_l0 = (1/rho)*z -(1/a.m)*a.k*(a.o-a.L) +(1/rho*1/a.m)*a.k*(a.k[a.index]*z[a.index]-multiply_vector(z,a.k);
    d_l0[a.index] = 0;

    sol_linear_0 = check_feasibility(a, d_l0);

    if (sol_linear_0==0) {
        cost_linear_0 = cost(a, d_l0, rho);
        if (cost_linear_0 < cost_best) {
            d_best = d_l0;
            cost_best = cost_linear_0;

        }

    }

    d_l1 = (1/rho)*z - (1/a.m)*a.k*(a.o-a.L+100*a.k[a.index]) + (1/rho*1/a.m)*a.k*(a.k[a.index]*z[a.index]-multiply_vector(z,k));
    d_l1[a.index] = 100;

    sol_linear_0 = check_feasibility(a, d_l1);

    if (sol_linear_0==0) {
        cost_linear_0 = evaluate_cost(node, d_l1, rho);
        if (cost_linear_0 < cost_best) {
            d_best = d_l1;
            cost_best = cost_linear_0;
        }
    }
    d = d_best;
    cost_in = cost_best;
}

int check_feasibility(node a, float d[]){
    float tol = 0.0001;

    if (d[a.index] < 0-tol) return 0;
    if (d[a.index] > 100+tol) return 0;
    if (multiply_vector(d,a.k) < a.L-a.o-tol), return 0;
    return= 1;
}

float cost(node a, float d[], float rho){
    float d[N]=sub_vector(d,a.d);
    float norm = norma(d_diff);
    float cost_aux=multiply_vector(a.c, d)+multiply_vector(a.y,d_diff)+rho/2*norm*norm;
    return cost_aux;
}

