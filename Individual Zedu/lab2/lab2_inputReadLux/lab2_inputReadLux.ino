
const int analogIn = A0;
float LDRvalue = 0.0;
float VR = 0.0;
float R = 0.0;
float b = 5.0;
float m = -0.7;
float R1 = 10000.0;
float L = 0.0;
float V = 5.0;


void setup() {
  Serial.begin(9600);
  pinMode(analogIn, INPUT);
}

void loop () {

  LDRvalue = analogRead(analogIn);
  
  VR = 5- ( LDRvalue / 255) ;

  R = (R1*VR) / (V-VR);
  
  L = pow(10, (log(R) / log(10) - b)/(m));
  
  Serial.print("valor LDR: ");
  Serial.print(LDRvalue);
  Serial.print(  " = ");
  Serial.print(L);
  Serial.println( " lux ");  
  
  delay(500);
  
  
  
}
