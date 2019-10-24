//
// Created by Alexandre Bernardino on 20/10/2019.
//

#include "pid.h"
#include <Arduino.h>
using namespace std;

void pid::init(float p, float i, float d, float _T, float _a)
{
    ep = yp = ip = dp = 0.0f;
    k1=kp=p; ki=i; kd=d; T=_T; a=_a;
    k2=kp*ki*T/2;
    if (kd < 0.000001f)
        derivative = false;
    else {
        float den = kd+a*T;
        //warning: should check den
        k3=kd/den;
        k4=kp*kd*a/den;
    }
}

float pid::calc(float ref, float y)
{
    //warning: code not optimized
    float e = ref - y;
    float p = k1*e;
    float i = ip + k2*(e+ep);
    float d = 0;
    if(derivative)
        d = k3*dp - k4*(y-yp);
    yp = y; ip = i; dp = d; ep = e;
    return p+i+d;
}
void pid::imprimir()
{
    Serial.print("kp: ");
    Serial.print(kp);
    Serial.print(" ki: ");
    Serial.print(ki);
    Serial.print(" kd: ");
    Serial.print(kd);
    Serial.print(" T: ");
    Serial.print(T);
    Serial.print(" a: ");
    Serial.println(a);
}
