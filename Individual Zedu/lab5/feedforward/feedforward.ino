

//read from ldr
const int analogIn = A0;
float LDRvalue = 0.0;
float VR = 0.0;
float R = 0.0;
float b = 4.8;
float m = -0.7;
float R1 = 10000.0;
float L = 0.0;
float L_des = 0.0;
float V = 5;
float a= 1.129;
float c= -6.47;

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

  Serial.println("Insira Lux Desejado");
  while (readIn == ""){
    readIn=Serial.readString();
    duty=0;
  }


  L_des = readIn.toInt();
  duty=(L_des-c)/a;
  readIn = "";

  
  duty_float = duty / 100.0;
  on = totalCiclo*duty_float;

  analogWrite(digitalOutLED, on);

  
//ler lux final
  delay(10000);
  LDRvalue = analogRead(analogIn);
  
  VR = 5- ( LDRvalue / 1023)*5.0 ;

  R = (R1*VR) / (V-VR);
  
  L = pow(10, (log(R) / log(10) - b)/(m));
  
  Serial.print("valor LDR desejado: ");
  Serial.print(L_des);
  Serial.println( " lux ");  
  
  Serial.print("valor LDR lido: ");
  Serial.print(L);
  Serial.println( " lux ");  



  delay(500);
  
  
  
}
