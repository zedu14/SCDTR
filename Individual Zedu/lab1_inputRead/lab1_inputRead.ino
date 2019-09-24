
const int digitalOutLED = 8;
int LEDIntens = 0;
int totalCiclo = 1000;
int duty = 0;
int on = 0;
int off = 0;
String readIn;

void setup() {
  Serial.begin(9600);
}

void loop () {

  readIn=Serial.readStringUntil('/n');
  
  duty = toInt(readIn);
  Serial.print("valor lido: ");
  Serial.println(duty);
  
  delay(100);
  
  
  
}
