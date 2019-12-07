#include "controlo.h"

#define ARDUINO 2

float L_des={0.0,0.0,0.0};
float 0=0.0;
float cost[N]={1.0,1.0,1.0};
float K[N]={2.0,1.0,1.0};
float d_ff=0.0;

void setup(){
d_ff=controlo-distribuido(L_des[ARDUINO], O, cost[ARDUINO],K, ARDUINO);

}

void loop(){

 
 
 
 
}
