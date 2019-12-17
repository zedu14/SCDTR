#ifndef PID_H
#define PID_H
#include <Arduino.h>
using namespace std;
class pid  {
    float kp, ki, T;
    float k1, k2;
    float ip;
public:
    void init(float kp, float ki, float T);
    float calc(float ref, float y, int u_feedforward);
    void imprimir();
};

#endif  //PID_H