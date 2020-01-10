#include "pid.h"
#include <Arduino.h>
using namespace std;
const int analogOutPin = 9;

void pid::init(float p, float i, float _T)
{
    ip = 0.0f;
    k1=kp=p; ki=i; T=_T;
    k2=kp*ki*T/2;


}

float pid::calc(float ref, float y, int u_feedforward)
{
    float e = ref - y;
    float p = k1*e;
    float i = ip + k2*(e);



    //anti-windup
    if(p+i+u_feedforward > 255)
      i = 255 - (p+u_feedforward);
    else if (p+i+u_feedforward < 0)
      i = -(-p+u_feedforward);

    analogWrite(analogOutPin, constrain(p+i+map(u_feedforward,0,100,0,255),0,255));
    ip = i;
    return p+i+u_feedforward;
}
void pid::imprimir()
{
    Serial.print("kp: ");
    Serial.print(kp);
    Serial.print(" ki: ");
    Serial.print(ki);
    Serial.print(" T: ");
    Serial.print(T);
}
