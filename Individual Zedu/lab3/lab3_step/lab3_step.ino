

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
int duty_init = 0;
float duty_float = 0.0;
float on = 0;
String readIn ="";

int i = 0;


void setup() {
  Serial.begin(9600);
  pinMode(analogIn, INPUT);
  pinMode(digitalOutLED, OUTPUT);
}

void loop () {

  Serial.println("Insira qual step");
  while(readIn == ""){
    readIn=Serial.readString();
    duty = duty_init;
    analogWrite(digitalOutLED, 0);  
  }

   duty = readIn.toInt();
  readIn = "";
  Serial.print("duty: ");
  Serial.println(duty);
  
  duty_float = duty / 100.0;
  on = totalCiclo*duty_float;


  analogWrite(digitalOutLED, on);  

  Serial.println("Out Lux: ");
  for (i=0;i<100;i++){
    LDRvalue = analogRead(analogIn);
    VR = 5- ( LDRvalue / 1023)*5 ;
    R = (R1*VR) / (V-VR);
    L = pow(10, (log(R) / log(10) - b)/(m));
    
    Serial.println(L); 
    delayMicroseconds(1); 
  }
 
    
   
  
  
  
}




