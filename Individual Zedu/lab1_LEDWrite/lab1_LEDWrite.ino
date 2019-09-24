
const int digitalOutLED = 8;
int LEDIntens = 0;
int totalCiclo = 1000;
int duty = 0;
float duty_float = 0.0;
float on = 0;
float off = 0;
String readIn ="";

void setup() {
  Serial.begin(9600);
}

void loop () {

  readIn=Serial.readString();
  
  duty = readIn.toInt();
  readIn = "";
  Serial.print("valor lido: ");
  Serial.println(duty);
  
  duty_float = duty / 100.0;
  on = totalCiclo*duty_float;
  off = totalCiclo - on;
  
  Serial.print("on durante ");
  Serial.print(on);
  Serial.print( " ms e off durante ");
  Serial.print(off);
  Serial.println(" ms");

  digitalWrite(digitalOutLED, HIGH);
  delay(on);  
  
  digitalWrite(digitalOutLED, LOW);
  delay(off);
  
  
  
}
