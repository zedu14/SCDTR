#ifndef COMS_H
#define COMS_H
#include <mcp2515.h> 

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

union my_can_msg { //to pack/unpack long ints into bytes 
  unsigned long value;
  unsigned char bytes[4];
};

void irqHandler();
MCP2515::ERROR write(uint32_t id, uint32_t val) ;
MCP2515::ERROR read(unsigned long &c);
int code_id(int order, int from, int to);
int decode_id(int id, int param);
float request(int ordem, int from, int to);
void send_data(int ordem, int valor, int from, int to);

#endif //CONTROLO_DISTRIBUIDO_CONTROLO_H
