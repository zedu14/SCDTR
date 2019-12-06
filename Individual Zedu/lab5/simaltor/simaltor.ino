

//read from ldr
const int analogIn = A0;
float LDRvalue = 0.0;
float VR = 0.0;
float R = 0.0;
float b = 4.8;
float m = -0.75;
float R1 = 10000.0;
float L = 0.0;
float L_des = 0.0;
float V = 5;
float a= 0.572;
float c= -2.812;

//write to led
const int digitalOutLED = 3;
int LEDIntens = 0;
int totalCiclo = 255;
int duty = 0;
float duty_float = 0.0;
float on = 0;
String readIn ="";

//simulador
float tau=0.0;
float k0=0.0;
float v_final=0.0;
float v_init=0.0;
float v_des=0.0;
float R_des=0.0;
float y_des=0.0;
float t=0.0;
float t_init=0.0;

float coef_ak=-0.04474;
float coef_bk=0.241;

float coef_atau=-38.636;
float coef_btau=172.95;

int i=0;





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

  
  tau=coef_atau*log(duty)+coef_btau;
  Serial.println(tau);
  tau=tau*pow(10,3);
  k0=coef_ak*log(duty)+coef_bk;
  Serial.println(k0);
  v_final=k0*duty;
  Serial.println(v_final);
  t_init=micros();
  Serial.println(t_init);
  v_init=analogRead(analogIn)*5/1023.0;
  Serial.println(v_init);
  
  Serial.println("Ciclo");
  
  for(i=0;i<500;i++){
    t=micros();
    v_des= v_final-(v_final-v_init)*exp(-(t-t_init)/tau);
    R_des=R1*(V/v_des-1)*1.0;
    y_des= pow(10, (log(R_des)/log(10)-b)/m);
    Serial.println(y_des);
    delayMicroseconds(1); 
  }
  
  
  
  
}
