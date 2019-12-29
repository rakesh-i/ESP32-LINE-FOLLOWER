#define MOT1 18
#define MOT2 5
#define MOT3 22
#define MOT4 23
#define PWM1 21
#define PWM2 19
#define LEDE 4 
int frequency = 3000;


void setup(){
  pinMode(MOT1,OUTPUT);
  pinMode(MOT2,OUTPUT);
  pinMode(MOT3,OUTPUT);
  pinMode(MOT4,OUTPUT);
  pinMode(PWM1,OUTPUT);
  pinMode(PWM2,OUTPUT);
  pinMode(LEDE,OUTPUT);
 
  ledcSetup(0,30000,8);
  ledcSetup(1,30000,8);

  ledcAttachPin(PWM1,0);
  ledcAttachPin(PWM2,1);
}


void loop(){
delay(5000);
while(1){
  digitalWrite(LEDE,HIGH);
  motorSpeed(300,-300);
  delay(180);
  motorSpeed(180,-180);
  delay(100);
  motorSpeed(0,0);
  digitalWrite(LEDE,LOW);
  delay(5000);
  digitalWrite(LEDE,HIGH);
  motorSpeed(300,-300);
  delay(410);
  motorSpeed(180,-180);
  delay(100);
  motorSpeed(0,0);
  break;
}
  
}


void motorSpeed(int M1,int M2){
  int pwm1=map(abs(M1),0,1000,0,255);
  int pwm2=map(abs(M2),0,1000,0,255);
  ledcWrite(0,pwm1);
  ledcWrite(1,pwm2);
  digitalWrite(MOT1,(M1>0) ? HIGH :LOW);
  digitalWrite(MOT2,(M1>0) ? LOW :HIGH);
  digitalWrite(MOT3,(M2>0) ? HIGH :LOW);
  digitalWrite(MOT4,(M2>0) ? LOW :HIGH);
  
}
