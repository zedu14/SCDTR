
const int analogIn = A0;
int LDRvalue = 0;


void setup() {
  Serial.begin(9600);
  pinMode(analogIn, INPUT);
}

void loop () {

  LDRvalue = analogRead(analogIn);
  
  Serial.print("valor LDR: ");
  Serial.println(LDRvalue);
  
  delay(100);
  
  
  
}
