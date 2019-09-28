
const int digitalOutLED = 3;
int LEDIntens = 0;
int totalCiclo = 255;
int duty = 0;
float duty_float = 0.0;
float on = 0;
String readIn ="";

void setup() {
  Serial.begin(9600);
  pinMode(digitalOutLED, OUTPUT);
}

void loop () {


  readIn=Serial.readString();
  
  if(readIn != "")
   duty = readIn.toInt();
  //readIn = "";
  Serial.print("valor lido: ");
  Serial.println(duty);
  
  duty_float = duty / 100.0;
  on = totalCiclo*duty_float;
  
  Serial.print("on durante ");
  Serial.print(on);
  Serial.println( " ms");

  analogWrite(digitalOutLED, on);

  
  
  
}
