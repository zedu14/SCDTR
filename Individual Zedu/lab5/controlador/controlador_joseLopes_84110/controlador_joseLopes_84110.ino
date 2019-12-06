

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
float a= 1.169;
float c= -6.47;

float Lmax=a*100+c;
float Lmin=c;

//write to led
const int digitalOutLED = 3;
int LEDIntens = 0;
int totalCiclo = 255;
int duty = 0;

float duty_total = 0.0;
float on = 0;
String readIn ="";

//simulador
float tau=0.0;
float k0=0.0;
float v_final=0.0;
float v_init=0.0;
float v_des=0.0;
float v_des_anterior=0.0;
float v_aux=0.0;
float R_des=0.0;
float y_des=0.0;
float t=0.0;
float t_antes=0.0;
float delta_t=0.0;
float t_init=0.0;

float coef_ak=-0.0619;
float coef_bk=0.3186;

float coef_atau=-28.6495;
float coef_btau=146.6582;

int i=0;

//controlador
float duty_fb=0.0;
float ui=0.0;
float ui_anterior=0.0;
float Ti=1000;
float k=3;
float e = 0.0;

//proportional term
float prop=0.0;

//integral term
float inte=0.0;

//read
float v_read=0.0;
float y=0.0;

int novo=0;



void setup() {
  Serial.begin(28800);
  pinMode(analogIn, INPUT);
  pinMode(digitalOutLED, OUTPUT);
}

void loop () {

  Serial.println("Esta alguem? [sim/nao]");
  while (readIn == ""){
    readIn=Serial.readString();
    if(readIn=="nao"){  
     L_des=Lmin;
     
    }
    else if(readIn=="sim"){
     Serial.println("Insira Lux Desejado");
     while (readIn == "sim"||readIn == ""){
       analogWrite(digitalOutLED,0);
       readIn=Serial.readString();
     }
    }
    else
     readIn="";
   
  }
  
  L_des=readIn.toInt(); 

for(novo=0;novo<1;novo=novo+1){
  
  L_des=L_des-60*novo; 
  duty=(L_des-c)/a;
  readIn = "";

  
  tau=coef_atau*log(L_des)/log(exp(1))+coef_btau;
  Serial.print(tau);
  tau=tau*pow(10,3);
  R_des=pow(10,(log(L_des)/log(10)*m+b));
  v_final=5.0*(R1/(R1+R_des));
  

  Serial.print("    ");
 Serial.print(k0);
     Serial.print("    ");
    Serial.print(duty);
    Serial.print("    ");
    Serial.println(v_final);
    
  t_init=micros();
  v_init=analogRead(analogIn)*5/1023.0;

  
  Serial.println("Desired    Experimental");
  
  for(i=0;i<500;i++){
    if(i>0)
     t_antes=t;
     
     
    t=micros();
    delta_t=t-t_antes;

    
    //ydes
    v_des_anterior=v_des;
    v_des= v_final-(v_final-v_init)*exp(-(t-t_init)/tau);


    
    R_des=(R1*(5.0/v_des-1.0))*1.0;
    y_des= pow(10, (log(R_des)/log(10)-b)/m);

    
    if(y_des>Lmax)
     y_des=Lmax;
    
    //y
    v_read=analogRead(analogIn)*5/1023.0;
    R=(R1*(5.0/v_read-1.0))*1.0;
    y= pow(10, (log(R)/log(10)-b)/m);
    
    //erro and duty_fb
    e=y_des-y;
    ui=(ui_anterior+delta_t/Ti*e)*k;
    ui_anterior=ui;
    prop=k*e;
    inte=ui;
    duty_fb=prop+inte;

    
    
    //saturar termo integral
    if(duty+prop+inte>100){
      ui=100-duty-prop;
      
    }
      
    if(duty+prop+inte<=0){
      ui=0-duty-prop;
    }
    
    inte=ui;
    duty_fb=prop+inte;
    

    
    
    duty_total = (duty+duty_fb) / 100.0;
    if(duty_total>100.0)
      duty_total = 100.0;
    else if(duty_total<=0)
      duty_total = 0.0;
    
    on = totalCiclo*duty_total;
    analogWrite(digitalOutLED,on);
    
    
    Serial.print(y);
    Serial.print("    ");
    Serial.print(y_des);
    Serial.print("    ");
    Serial.println(delta_t);
    


    
    delayMicroseconds(1); 
  }
  
}
  
  
}
