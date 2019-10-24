//
// Created by Alexandre Bernardino on 20/10/2019.
//

#ifndef PID_H
#define PID_H
#include <Arduino.h>
using namespace std;
class pid  {
    bool derivative;
    float kp, ki, kd, T, a;
    float k1, k2, k3, k4;
    float ep, yp, ip, dp;
public:
    void init(float kp, float ki,
              float kd, float T, float a);
    float calc(float ref, float y);
    void imprimir();
};

#endif  //PID_H
