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