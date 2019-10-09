

//read from ldr
const int analogIn = A0;
float LDRvalue = 0.0;
float VR = 0.0;
float R = 0.0;
float b = 4.9;
float m = -0.84;
float R1 = 10000.0;
float L = 0.0;
float V = 5;

//write to led
const int digitalOutLED = 3;
int LEDIntens = 0;
int totalCiclo = 255;
int duty = 0;
float duty_float = 0.0;
float on = 0;
String readIn ="";


void setup() {
  Serial.begin(9600);
  pinMode(analogIn, INPUT);
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

  analogWrite(digitalOutLED, on);


  LDRvalue = analogRead(analogIn);
  
  VR = 5- ( LDRvalue / 1023)*5 ;

  R = (R1*VR) / (V-VR);
  
  L = pow(10, (log(R) / log(10) - b)/(m));
  
  Serial.print("valor LDR: ");
  Serial.print(LDRvalue);
  Serial.print(  " = ");
  Serial.print(L);
  Serial.println( " lux ");  
  
  /*Serial.print("valor R: ");
  Serial.println(R);*/


  delay(500);
  
  
  
}





