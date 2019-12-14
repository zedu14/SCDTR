#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h> 
#include "math.h"
#include "pid.h"

//alterar consoante o arduino a carregar
#define ARDUINO 3
//numero de arduinos no sistema
#define N 3
#define MaxIter 50

#define ENVIAR_DAV_OUTROS 10
#define PEDIR_DAV_OUTROS 11
#define ENVIAR_D_PARA_AV 12
#define PEDIR_D_PARA_AV 13
#define NEW_DFF 37


float pinOut = 9;
float sensorPin = A0; // select the input pin for LDR
float sensorValue = 0.0; // variable to store the value coming from the sensor
float V0 = 0.0;
float I = 0.0; 
float R_LDR = 0.0;
float Lux = 0.0;
float b = 4.8;
float m = -0.73;
float l_bound_occupied = 20.0;
float l_bound_empty = 0.0;
float cost = 1.0;
float d_ff=0.0;
int n_calibrate = 0;
int flag_occupied = 0;
int flag_atualiza_dff=0;

int duty = 0;

String serialIn ="";

float O=0.0;
float K[N]={0.0};

uint32_t mask = 0x00000003;
uint32_t filt = ARDUINO;

pid controller;
float ganho_p=10.0; 
float ganho_i=10.0;
float fs=100.0;
float Tamostragem=1/fs;
float R1=10000.0;
float t_init=0.0;
float v_init=0.0;
float coef_btau[N]={87900.0,83500.0,65655.0};
float coef_atau[N]={-122000,-12000.0,-12141.0};
float y_des=0.0;
float y=0.0;

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
class can_frame_stream {
  static constexpr int buffsize = 10; //space for 10 can_messages - increase if needed 
  can_frame cf_buffer[buffsize];
  int read_index; //where to read next message
  int write_index; //where to write next message
  bool write_lock; //buffer full 
public:
  can_frame_stream() : read_index{0}, write_index{0}, write_lock{false} {}; 
  int put(can_frame &frame) {
    if(write_lock) return 0; //buffer full
    cf_buffer[write_index] = frame; 
    write_index=(++write_index)%buffsize;
    if(write_index == read_index) write_lock = true; //cannot write more 
    return 1;
  }
  int get(can_frame &frame) {
    if(!write_lock && (read_index==write_index) ) return 0; //empty buffer 
    if(write_lock && (read_index==write_index) ) write_lock = false; //release lock 
    frame = cf_buffer[read_index];
    read_index = (++read_index)%buffsize;
    return 1;
  }
} volatile cf_stream; //the object to use

MCP2515 mcp2515(10); //SS pin 10
volatile bool interrupt = false; //notification flag for ISR and loop() 
volatile bool mcp2515_overflow = false;
volatile bool arduino_overflow = false;

void irqHandler() {
  can_frame frame;
  uint8_t irq = mcp2515.getInterrupts(); //read CANINTF
  if (irq & MCP2515::CANINTF_RX0IF) { //msg in receive buffer 0
    mcp2515.readMessage(MCP2515::RXB0, & frame); //also clears RX0IF
    if(!cf_stream.put(frame)) arduino_overflow = true; 
    }
    if (irq & MCP2515::CANINTF_RX1IF) { //msg in receive buffer 1 
      mcp2515.readMessage(MCP2515::RXB1, & frame); //also clears RX1IF
      if(!cf_stream.put(frame)) arduino_overflow = true; 
    }
    irq = mcp2515.getErrorFlags(); //read EFLG
    if( (irq & MCP2515::EFLG_RX0OVR) | (irq & MCP2515::EFLG_RX1OVR) ) {
    mcp2515_overflow = true;
    mcp2515.clearRXnOVRFlags(); 
    }
    mcp2515.clearInterrupts();
    interrupt = true; //notify loop() 
    }

union my_can_msg { //to pack/unpack long ints into bytes 
  unsigned long value;
  unsigned char bytes[4];
};

MCP2515::ERROR write(uint32_t id, uint32_t val) { 
  can_frame frame;
  frame.can_id = id;
  frame.can_dlc = 4;
  my_can_msg msg;
  msg.value = val;
  for(int i = 0; i < 4; i++) frame.data[i] = msg.bytes[i];
  return mcp2515.sendMessage(&frame);
}

MCP2515::ERROR read(unsigned long &c) {
  can_frame frame;
  my_can_msg msg;
  MCP2515::ERROR err = mcp2515.readMessage(&frame); 
  if(err == MCP2515::ERROR_OK) {
    for(int i = 0; i < 4; i++) msg.bytes[i] = frame.data[i];
    c = msg.value; 
    }
  return err; 
}
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
        cost_unconstrained = cost_arduino(a, d_u, rho);
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
        cost_boundary_linear = cost_arduino(a, d_bl, rho);
        if (cost_boundary_linear < cost_best) {
            equal_vector(d_best,d_bl,N);
            cost_best = cost_boundary_linear;
        }
    }

    multiply_c_vector(1.0/rho,z,d_b0,N);
    d_b0[a.index] = 0.0;

    sol_boundary_0 = check_feasibility(a, d_b0);

    if (sol_boundary_0 == 1) {
        cost_boundary_0 = cost_arduino(a, d_b0, rho);
        if (cost_boundary_0 < cost_best) {
            equal_vector(d_best,d_b0,N);
            cost_best = cost_boundary_0;
        }
    }

    multiply_c_vector(1.0/rho,z,d_b1,N);
    d_b1[a.index] = 100.0;

    sol_boundary_100 = check_feasibility(a, d_b1);

    if (sol_boundary_100==1){
        cost_boundary_100 = cost_arduino(a, d_b1, rho);
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
        cost_linear_0 = cost_arduino(a, d_l0, rho);
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
        cost_linear_100 = cost_arduino(a, d_l1, rho);
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

float cost_arduino(node a, float d[], float rho){
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





 float update_self_av(int i, float d[]){
  int j=0;
  int k=0;
  int aux=0;
  int from,ordem,to=-1;
  float dji=0.0;
  float av=0.0;
  can_frame frame;
  my_can_msg msg;
     

  for(j=0;j<N;j++){
   if(i!=j){
    //ver se o j já pediu que lhe envie o que acho que ele é
    while(ordem != PEDIR_D_PARA_AV && from != j+1){ 
      while(cf_stream.get(frame)){
      ordem = decode_id(frame.can_id,1);
      from = decode_id(frame.can_id,2);
      }
    }

    //se j ja pediu, entao envia
    send_data(ENVIAR_D_PARA_AV, d[j]*100.0, i+1, j+1);
    delay(100);

    ordem=-1;
    from=-1;
   }
   else{
    //entao vamos calcular o meu
    for (k=0;k<N;k++){
      if(k!=i){
        //pedir ao k que envie o seu
        send_data(PEDIR_D_PARA_AV, 1, i+1, k+1); 
        delay (100);

        //esperar que k envie o que pedi
        while(ordem != ENVIAR_D_PARA_AV && from != k+1){ 
          while(cf_stream.get(frame)){
          ordem = decode_id(frame.can_id,1);
          from = decode_id(frame.can_id,2);
        } 
      }
      for(aux = 0; aux < 4; aux++) msg.bytes[aux] = frame.data[aux];       

      dji = msg.value/100.0;
      av+=dji; 
    }
    else{
     av+=d[i];
    }
   }
   av=av/N;
   }
  }
  return av;
 }
   
 
 void update_geral_av(int i, float av[], float d[]){
  int j=0;
  int k=0;
  int aux=0;
  int from,ordem,to=-1;
  float dji=0.0;
  can_frame frame;
  my_can_msg msg;
     

  for(j=0;j<N;j++){
   if(i!=j){
    //ver se o j já pediu que lhe envie a minha media
    // ENVIAR_DAV_OUTROS 10
    // PEDIR_DAV_OUTROS 11
    while(ordem !=  PEDIR_DAV_OUTROS && from != j+1){ 
      while(cf_stream.get(frame)){
      ordem = decode_id(frame.can_id,1);
      from = decode_id(frame.can_id,2);
      }
    }

    //se j ja pediu, entao envia
    send_data(ENVIAR_DAV_OUTROS, av[i]*100.0, i+1, j+1);
    delay(100);

    ordem=-1;
    from=-1;
   }
   else{
    //agora vamos preencher as outras posicoes com os deles porque o meu ja ta
    for (k=0;k<N;k++){
      if(k!=i){
        //pedir ao k que envie o seu
        send_data(PEDIR_DAV_OUTROS, 1, i+1, k+1); 
        delay (100);

        //esperar que k envie o que pedi
        while(ordem != ENVIAR_DAV_OUTROS && from != k+1){ 
          while(cf_stream.get(frame)){
          ordem = decode_id(frame.can_id,1);
          from = decode_id(frame.can_id,2);
          } 
        }      

        for(aux = 0; aux < 4; aux++) msg.bytes[aux] = frame.data[aux];
        dji= msg.value/100.0; 
        av[k] = dji;
      }
    }
   }
  }
 }






float controlo_distribuido(float L, float O, float c, float k[], int i){

  int iter=0;
  float custo=0.0;
  float rho=0.07;
  int j=0;
  node no_arduino;
  no_arduino.index=i;
  for(j=0;j<N;j++){
    no_arduino.d[j]=0.0;
    no_arduino.d_av[j]=0.0;
    no_arduino.y[j]=0.0;
    no_arduino.k[j]=0.0;
    no_arduino.c[j]=0.0;
  }
  
  equal_vector(no_arduino.k,k,N); 
  no_arduino.n=norma_2(no_arduino.k, N);
  no_arduino.m=no_arduino.n-no_arduino.k[i]*no_arduino.k[i];
  no_arduino.c[i]=c;
  no_arduino.o=O;
  no_arduino.L=L;

  

    
  for(iter=1;iter<MaxIter;iter++){
    custo=consensus_iterate(no_arduino.d, no_arduino, rho);
 Serial.print("Iteracao : ");
 Serial.println(iter);
     //computar media
    no_arduino.d_av[i]=update_self_av(i, no_arduino.d);
    update_geral_av(i,no_arduino.d_av, no_arduino.d);
    //atualizar lagrange
    
    update_lagrange(no_arduino,rho);
  }
  

  Serial.print("dff: ");
  for(j=0;j<N;j++){
      Serial.println(no_arduino.d_av[j]);
  }
  return no_arduino.d_av[i];
}

void atualiza_dff(){
  
  if(ARDUINO==N){
    duty=duty-d_ff;
    for(int j=1;j<N;j++)
    send_data(NEW_DFF, 1, ARDUINO, j);
    d_ff= controlo_distribuido(Lux, O, cost, K, ARDUINO-1);
    duty+=d_ff;   
    }  
}

int code_id(int order, int from, int to){
  return 16*order + 4*from + to;
}

int decode_id(int id, int param){
  int obj = 0;
  //order
  if(param == 1){
    obj = (id & 2032)/16;
  }
  //from
  else if (param == 2){
    obj = (id & 12)/4;
  }
  //to
  else if (param == 3)
  {
    obj = id & 3;
  }
  return obj;
}

float bit_to_lux(){
  sensorValue = analogRead(sensorPin); // read the value from the sensor
  V0 = (5.0000/1023.0)*sensorValue;
  I=V0/10000;
  R_LDR=(5.0-V0)/I;
  Lux=pow(10, (log10(R_LDR)-b)/(m));
  return Lux;
}

float request(int ordem, int from, int to){
  can_frame frame;
  my_can_msg msg;
  unsigned long t_init = 0;
  int id = 0;
  id = code_id(ordem,from,to);
  //Faz o request
  if( write(id,0) != MCP2515::ERROR_OK)
    Serial.println("\t\tError: MCP2515 TX Buffers Full");
  
  //Espera pela mensagem
  t_init = micros();
  //Serial.println("\tWaiting for Data");
  while(interrupt == false){Serial.print("\tWaiting for Data [us] : ");Serial.println(micros()-t_init);}
  cf_stream.get(frame);
  //le mensagem
  interrupt = false;
  for(int i = 0; i < 4; i++) msg.bytes[i] = frame.data[i]; 
  Serial.print("\tReceiving: "); Serial.println(msg.value);
  return msg.value;
}

void send_data(int ordem, int valor, int from, int to){
  int id = code_id(ordem,from,to);
  if( write(id,valor) != MCP2515::ERROR_OK)
    Serial.println("\t\tError: MCP2515 TX Buffers Full");
}

void imprimir_k_o(){
  int m=0;

  Serial.print("O : ");Serial.println(O);

  for(m=0;m<3;m++){
    Serial.print("K");Serial.print(ARDUINO-1);Serial.print(m);Serial.print(": ");Serial.println(K[m]);
  }
}

void calibrate(){
  int i=0;

  Serial.println("Calibrating...");
  delay(1000);
  O=bit_to_lux();
  delay(1000);
  analogWrite(pinOut, 255);
  delay(1000);
  K[ARDUINO-1]=(bit_to_lux()-O)/100.0;
  analogWrite(pinOut, 0);
  delay(1000);
  //set duty of node 1 to 255
  for(i=1;i<N;i++){
    if(i==ARDUINO)
      continue;
    else{
      for(i=1;i<=N;i++){
        if(i == ARDUINO)
          continue;
        send_data(30,255,ARDUINO,i);
        delay(1000);
        K[i-1] = (bit_to_lux()-O)/100;
        send_data(30,0,ARDUINO,i);
        delay(1000); 
      }
    }
  }
  imprimir_k_o();
  //caso nao seja o ultimo arduino, notifica o proximo para calibrar
  if(ARDUINO != N)
    send_data(0,0,ARDUINO,ARDUINO+1); 
}
void hub(){
  serialIn = Serial.readString();
  char command = serialIn.charAt(0);
  char variable = "";
  String value_ = "";
  int node = 0;
  int flag_self = 0;
  int len = 0;
  float value = 0.0;

  node = serialIn.charAt(3)-'0';
  len = serialIn.length();
  value_ = serialIn.substring(7,len-1);
  value = value_.toFloat();
  if (node == ARDUINO)
   flag_self = 1;
   
  switch (command) {
    //requests
    case 'g':
      variable = serialIn.charAt(2);
      node = serialIn.charAt(5)-'0';
      if (node == ARDUINO)
        flag_self = 1;
      switch (variable){
        case 'l': //luminancia
          if(flag_self)
            Serial.println(bit_to_lux());
          else
            Serial.println("l <"+String(node)+"> "+"<"+String(request(1,ARDUINO,node)/100.0)+">");
            //Serial.println(request(1,ARDUINO,node)/100.0);
        break;
        case 'd': //duty
        if(flag_self ==1)
          Serial.println(duty);
        else
          Serial.println("d <"+String(node)+"> "+"<"+String(request(2,ARDUINO,node))+">");
        break;
        case 'o': //occupancy
        if(flag_self ==1)
          Serial.println(flag_occupied);
        else
          Serial.println("o <"+String(node)+"> "+"<"+String(request(3,ARDUINO,node))+">");
          //Serial.println(request(3,ARDUINO,node));
        break;  
        case 'O': //illuminance lower bound occupied
        if(flag_self ==1)
          Serial.println(l_bound_occupied);
        else
          Serial.println("O <"+String(node)+"> "+"<"+String(request(4,ARDUINO,node))+">");
          //Serial.println(request(4,ARDUINO,node));
        break;  
        case 'U': //illuminance lower bound occupied
        if(flag_self ==1)
          Serial.println(l_bound_empty);
        else
          Serial.println("U <"+String(node)+"> "+"<"+String(request(5,ARDUINO,node))+">");
          //Serial.println(request(5,ARDUINO,node));
        break;  
      }
      break;
    //set occupancy
    case 'o':
      if(flag_self)
        flag_occupied = value;
      else
        send_data(20,value,ARDUINO,node);

      flag_atualiza_dff=1;
    break;
    //set illuminance lower bound for occupied state
    case 'O':
      if(flag_self)
        l_bound_occupied = value;
      else
        send_data(21,value,ARDUINO,node);
      
      if(flag_occupied==1)
        flag_atualiza_dff=1;
    break;
    //set illuminance lower bound for empty state
    case 'U':
      if(flag_self)
        l_bound_empty = value;
      else
        send_data(22,value,ARDUINO,node);
      
      if(flag_occupied==0)
        flag_atualiza_dff=1;
    break;
    //set cost of energy
    case 'c':
      if(flag_self)
        cost = value;
      else
        send_data(23,value,ARDUINO,node);

      flag_atualiza_dff=1;
    break;
    //funcoes personalizadas
    case 's':
      if(flag_self){
        duty = value;
        analogWrite(pinOut,duty);
      }
      else
        send_data(30,value,ARDUINO,node);
    break;
  }
}

void process_order(int ordem, int value, int from, int to){
  if(ordem == 0){
    calibrate();

    if(ARDUINO==N)
      flag_atualiza_dff=1;
    //apos a calibracao do ultimo arduino, calcula-se d_ffs
  }
  //requests
  else if(ordem == 1)
    send_data(1,bit_to_lux()*100,to, from);
  else if (ordem ==2)
    send_data(2,duty,to, from);
  else if (ordem ==3)
    send_data(3,flag_occupied,to, from);
  else if (ordem ==4)
    send_data(4,l_bound_occupied,to, from);
  else if (ordem ==5)
    send_data(5,l_bound_empty,to, from);
  //sets
  else if (ordem ==20)
    flag_occupied = value;
  else if (ordem ==21)
    l_bound_occupied = value;
  else if (ordem ==22)
    l_bound_empty = value;
  else if (ordem ==23)
    cost = value;
  //funcoes personalizadas
  else if(ordem == 30){
    duty = value;
    analogWrite(pinOut,duty);
  }
  else if(ordem == NEW_DFF){
    duty=duty-d_ff;
    d_ff= controlo_distribuido(Lux, O, cost, K, ARDUINO-1);
    duty+=d_ff;
  }
    
}
unsigned long counter = 0;

void setup() {
  Serial.begin(250000);
  SPI.begin();
  attachInterrupt(0, irqHandler, FALLING); //use interrupt at pin 2 
  //Must tell SPI lib that ISR for interrupt vector zero will be using SPI 
  SPI.usingInterrupt(0);
  mcp2515.reset(); 
  mcp2515.setBitrate(CAN_1000KBPS, MCP_16MHZ); 

  mcp2515.setFilterMask(MCP2515::MASK0, 0, mask);
  mcp2515.setFilterMask(MCP2515::MASK1, 0, mask);
  //filters related to RXB0
  mcp2515.setFilter(MCP2515::RXF0, 0, filt); mcp2515.setFilter(MCP2515::RXF1, 0, filt);
  //filters related to RXB1
  mcp2515.setFilter(MCP2515::RXF2, 0, filt); mcp2515.setFilter(MCP2515::RXF3, 0, filt);
  mcp2515.setFilter(MCP2515::RXF4, 0, filt); mcp2515.setFilter(MCP2515::RXF5, 0, filt);
  
  
  mcp2515.setNormalMode();
  //mcp2515.setLoopbackMode();
  if (ARDUINO == 1){
    delay(1000);
    calibrate();
  }
  controller.init(ganho_p, ganho_i,Tamostragem);
      
  t_init=micros();
  v_init=analogRead(sensorPin)*5/1023.0;
}


float simulador(float t){
  float v_des=0.0;
  float R_des=0.0;
  float tau=0.0;
  float v_final=0.0;
  
  tau=coef_atau[ARDUINO-1]*log(d_ff)/log(exp(1))+coef_btau[ARDUINO-1];
  tau=tau*pow(10,3);
  R_des=pow(10,(log(Lux)/log(10)*m+b));
  v_final=5.0*(R1/(R1+R_des));
  
  //ydes
  v_des= v_final-(v_final-v_init)*exp(-(t-t_init)/tau);
  
  R_des=(R1*(5.0/v_des-1.0))*1.0;
  y_des= pow(10, (log(R_des)/log(10)-b)/m);
  return y_des;
}

void loop() {
  int ordem,from,to,ativa_envia_lux=0;
  float t_i_loop=0.0;
  
  if(flag_occupied == 0)
    Lux=l_bound_occupied;
  else
    Lux=l_bound_empty;

  
  if(Serial.available())
    hub();
     
  if(flag_atualiza_dff==1){
    atualiza_dff();
    flag_atualiza_dff=0;
  }

  if(interrupt) {
    interrupt = false; 
    if(mcp2515_overflow) {
      Serial.println("\t\tError: MCP2516 RX Buffers Overflow");
      mcp2515_overflow = false; 
      }
    if(arduino_overflow) {
      Serial.println("\t\tError: Arduino Stream Buffers Overflow");
      arduino_overflow = false; 
      }
    can_frame frame;
    while( cf_stream.get(frame) ) {
      my_can_msg msg;
      for(int i = 0; i < 4; i++) msg.bytes[i] = frame.data[i]; 
      Serial.print("\tReceiving: "); Serial.println(msg.value);
      ordem = decode_id(frame.can_id,1);
      from = decode_id(frame.can_id,2);
      to = decode_id(frame.can_id,3);
      Serial.print("  ID: "); Serial.print(frame.can_id);Serial.print(" ");Serial.print(ordem);Serial.print(" ");Serial.print(from);Serial.print(" ");Serial.println(to);

      process_order(ordem, msg.value, from, to);
    }  
  }

  t_i_loop=micros();
  y_des=simulador(micros());
  y=bit_to_lux();

  controller.calc(y_des, y, d_ff);
  if(micros()-t_i_loop <Tamostragem)
    delayMicroseconds(Tamostragem*pow(10,6)-(micros()-t_i_loop)); 
}
