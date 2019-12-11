#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h> 

//alterar consoante o arduino a carregar
#define ARDUINO 3
//numero de arduinos no sistema
#define N 3

float pinOut = 9;
float sensorPin = A0; // select the input pin for LDR
float sensorValue = 0.0; // variable to store the value coming from the sensor
float V0 = 0.0;
float I = 0.0; 
float R_LDR = 0.0;
float Lux = 0.0;
float b = 4.8;
float m = -0.73;
float l_bound_occupied = 0.0;
float l_bound_empty = 0.0;
float cost = 0.0;
int flag_occupied = 0;

int duty = 0;

String serialIn ="";

float O=0.0;
float K[N]={0.0};

uint32_t mask = 0x00000003;
uint32_t filt = ARDUINO;

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
            Serial.println(request(1,ARDUINO,node)/100.0);
        break;
        case 'd': //duty
        if(flag_self ==1)
          Serial.println(duty);
        else
          Serial.println(request(2,ARDUINO,node));
        break;
        case 'o': //occupancy
        if(flag_self ==1)
          Serial.println(flag_occupied);
        else
          Serial.println(request(3,ARDUINO,node));
        break;  
        case 'O': //illuminance lower bound occupied
        if(flag_self ==1)
          Serial.println(l_bound_occupied);
        else
          Serial.println(request(4,ARDUINO,node));
        break;  
        case 'U': //illuminance lower bound occupied
        if(flag_self ==1)
          Serial.println(l_bound_empty);
        else
          Serial.println(request(5,ARDUINO,node));
        break;  
      }
      break;
    //set occupancy
    case 'o':
      if(flag_self)
        flag_occupied = value;
      else
        send_data(20,value,ARDUINO,node);
    break;
    //set illuminance lower bound for occupied state
    case 'O':
      if(flag_self)
        l_bound_occupied = value;
      else
        send_data(21,value,ARDUINO,node);
    break;
    //set illuminance lower bound for empty state
    case 'U':
      if(flag_self)
        l_bound_empty = value;
      else
        send_data(22,value,ARDUINO,node);
    break;
    //set cost of energy
    case 'c':
      if(flag_self)
        cost = value;
      else
        send_data(23,value,ARDUINO,node);
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
  if(ordem == 0)
    calibrate();
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
}


void loop() {
  int ordem,from,to,ativa_envia_lux=0;
  if(Serial.available())
    hub();

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
}
