#include <SPI.h>
#include <mcp2515.h> 
float pinOut = 9;
float sensorPin = A0; // select the input pin for LDR
float sensorValue = 0.0; // variable to store the value coming from the sensor
float V0 = 0.0;
float I = 0.0; 
float R_LDR = 0.0;
float Lux = 0.0;
float b = 4.8;
float m = -0.73;
float Lux1, Lux2;


float O[3]={};
float K[3][3]={};

uint32_t mask = 0xFFFFFFFF;
uint32_t filt0 = 0x01; //accepts msg ID 1 on RXB0
uint32_t filt1 = 0x02; //accepts msg ID 2 on RXB1
uint32_t filt2 = 0xF3; //accepts msg ID 3 on RXB1

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

MCP2515::ERROR write_lux(uint32_t id, int val) { 
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

float bit_to_lux(){
  sensorValue = analogRead(sensorPin); // read the value from the sensor
  V0 = (5.0000/1023.0)*sensorValue;
  I=V0/10000;
  R_LDR=(5.0-V0)/I;
  Lux=pow(10, (log10(R_LDR)-b)/(m));
  return Lux;
}

void ler_deux(){
  can_frame frame;
  while( cf_stream.get(frame) ) {
    my_can_msg msg;
    for(int i = 0; i < 4; i++) msg.bytes[i] = frame.data[i]; 
    Serial.print("\tReceiving: "); Serial.print(msg.value);
    Serial.print("  ID: "); Serial.println(frame.can_id/16);
    if (frame.can_id/16==1)
      Lux1=msg.value;
    if (frame.can_id/16==2)
      Lux2=msg.value; 
  }
}

void calibrate(){
  int u1,u2,u3=0;
  O[2]=bit_to_lux();
  ler_deux();
  O[0]=Lux1;
  O[1]=Lux2;
  
  delay(500);

  analogWrite(pinOut, 255);
  delay(500);
  K[2][2]=bit_to_lux()-O[2];
  ler_deux();
  K[0][2]=Lux1 - O[0];
  K[1][2]=Lux2 - O[1];

  analogWrite(pinOut, 0);

  if( write_lux(0x31,255) != MCP2515::ERROR_OK)
    Serial.println("\t\tError: MCP2515 TX Buffers Full");
  delay(500);
  K[2][0]=bit_to_lux()-O[2];
  ler_deux();
  K[0][0]=Lux1 - O[0];
  K[1][0]=Lux2 - O[1];

  if( write_lux(0x31,0) != MCP2515::ERROR_OK)
    Serial.println("\t\tError: MCP2515 TX Buffers Full");
  delay(500);

   
  if( write_lux(0x32,255) != MCP2515::ERROR_OK)
    Serial.println("\t\tError: MCP2515 TX Buffers Full");
  delay(500);
  K[2][1]=bit_to_lux()-O[2];
  ler_deux();
  K[0][1]=Lux1 - O[0];
  K[1][1]=Lux2 - O[1];

  if( write_lux(0x32,0) != MCP2515::ERROR_OK)
    Serial.println("\t\tError: MCP2515 TX Buffers Full");
  
}

unsigned long counter = 0;

void setup() {
  Serial.begin(115200);
  SPI.begin();
  attachInterrupt(0, irqHandler, FALLING); //use interrupt at pin 2 //Must tell SPI lib that ISR for interrupt vector zero will be using SPI 
  SPI.usingInterrupt(0);
  mcp2515.reset(); 
  mcp2515.setBitrate(CAN_1000KBPS, MCP_16MHZ); 

  mcp2515.setFilterMask(MCP2515::MASK0, 0, mask);
  mcp2515.setFilterMask(MCP2515::MASK1, 0, mask);
  //filters related to RXB0
  mcp2515.setFilter(MCP2515::RXF0, 0, filt2); mcp2515.setFilter(MCP2515::RXF1, 0, filt2);
  //filters related to RXB1
  mcp2515.setFilter(MCP2515::RXF2, 0, filt2); mcp2515.setFilter(MCP2515::RXF3, 0, filt2);
  mcp2515.setFilter(MCP2515::RXF4, 0, filt2); mcp2515.setFilter(MCP2515::RXF5, 0, filt2);
  
  
  mcp2515.setNormalMode();
  //mcp2515.setLoopbackMode();
  calibrate();
}

void loop() {

  /*sensorValue = analogRead(sensorPin); // read the value from the sensor
  V0 = (5.0000/1023.0)*sensorValue;
  I=V0/10000;
  R_LDR=(5.0-V0)/I;
  Lux=pow(10, (log10(R_LDR)-b)/(m));

   
  
  can_frame frame;
  while( cf_stream.get(frame) ) {
    my_can_msg msg;
    for(int i = 0; i < 4; i++) msg.bytes[i] = frame.data[i]; 
    Serial.print("\tReceiving: "); Serial.print(msg.value);
    Serial.print("  ID: "); Serial.println(frame.can_id/16);
  }
  Serial.print(" ID3 Lux: "); Serial.println(Lux);
  delay(500);

  
  /*
  //for(int i = 0; i < 11; i++) { //send several msgs in a burst
  
  //Serial.print("Sending: "); 
  //Serial.println(counter); 
  if( write(1,3) != MCP2515::ERROR_OK)
    Serial.println("\t\tError: MCP2515 TX Buffers Full");
  //if( write(i, counter++) != MCP2515::ERROR_OK)
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
  }*/
  
}
