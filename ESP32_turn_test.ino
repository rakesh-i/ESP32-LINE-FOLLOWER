#include<SimplyAtomic.h>
#include<Arduino.h>

class SimplePID{
  private:
    float kp, kd, ki, umax; 
    float eprev, eintegral; 
  public:
  SimplePID() : kp(1), kd(0), ki(0), umax(200), eprev(0.0), eintegral(0.0){}
  void setParams(float kpIn, float kdIn, float kiIn, float umaxIn){
    kp = kpIn; kd = kdIn; ki = kiIn; umax = umaxIn;
  }
  void evalu(int value, int target, float deltaT, int &pwr, int &dir){
    int e = target - value;
    float dedt = (e-eprev)/(deltaT);
    eintegral = eintegral + e*deltaT;
    float u = kp*e + kd*dedt + ki*eintegral;
    pwr = (int) fabs(u);
    if( pwr > umax ){
      pwr = umax;
    }
    dir = 1;
    if(u<0){
      dir = -1;
    }
    eprev = e;
  }
  
};

#define NMOTORS 2
const int led = 15;
const int enca[] = {4,14};
const int encb[] = {16,27};
const int pwm[] = {5,23};
const int in1[] = {19,21};
const int in2[] = {18,22};
long prevT = 0;
volatile int posi[] = {0,0};
#define LEDR 15 
const int WHITE = 1;
const int BLACK  = 0;
#define COLOR   BLACK    
const int N_SENS  =      6 ;     							
#define R_SENS        1000   								
#define WEIGHT_UNIT   1000    							
const int RIGHT_DIR =    1;
const int LEFT_DIR =    0;
#define CAL_SPEED     600  								
#define CAL_TIME      880     							
#define P_LINE_MIN    0.5     							
int sensRight = 26;
int sensLeft = 25;
const int SENSOR[N_SENS] = {33,32,35,34,39,36};     
int sens_min[N_SENS]; 
int sens_max[8];
float sens_scaled[6];
char path[100]="";
unsigned char path_length = 0;
unsigned char foundleft = 0;
unsigned char foundright = 0;
unsigned char foundstraight = 0;

//PID values
const float SPEED = 800;
const float KP = .2;
const float KD =5 ;
const float KI =.0001;
float line_pos = 0;
float last_line_pos = 0;


void readEncoder0(){
  int b = digitalRead(encb[0]);
  if(b > 0){
    posi[0] = posi[0]+1;
  }
  else{
    posi[0] =  posi[0]-1;
  }
}

void readEncoder1(){
  int b = digitalRead(encb[1]);
  if(b > 0){
    posi[1]++;
  }
  else{
    posi[1]--;
  }
}

SimplePID pid[NMOTORS];
int  pos[NMOTORS];

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2, int ch){
  ledcWrite(ch, pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}

void set(float p1, float p2, float cal){
  unsigned long ms = 0;
  ms =millis();
  while((ms+cal)>millis()){
  float target[NMOTORS];
  target[0] =p1;
  target[1] =p2;
  pos[0] = 0;
  pos[1] = 0;
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;
  ATOMIC(){
    for(int k = 0; k < NMOTORS; k++){
      pos[k] = posi[k];
    }
  }
  for(int k = 0; k < NMOTORS; k++){
    int pwr, dir;
    pid[k].evalu(pos[k],target[k],deltaT,pwr,dir);
    setMotor(dir,pwr,pwm[k],in1[k],in2[k],k);
    
  }
  for(int k = 0; k < NMOTORS; k++){
    Serial.print(target[k]);
    Serial.print(" ");
    Serial.print(pos[k]);
    Serial.print(" ");
    
  }
  Serial.println();
  }
  posi[0] = 0;
  posi[1] = 0;
}

void sets(float p1, float p2, float cal){
  unsigned long ms = 0;
  ms =millis();
  while((ms+cal)>millis()){
  float target[NMOTORS];
  target[0] =p1;
  target[1] =p2;
  pos[0] = 0;
  pos[1] = 0;
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;
  ATOMIC(){
    for(int k = 0; k < NMOTORS; k++){
      pos[k] = posi[k];
    }
  }
  for(int k = 0; k < NMOTORS; k++){
    int pwr, dir;
    pid[k].evalu(pos[k],target[k],deltaT,pwr,dir);
    setMotor(dir,pwr,pwm[k],in1[k],in2[k],k);
    
  }
  if (digitalRead(sensLeft)==0){
      foundleft = 1;
    }
  if(digitalRead(sensRight)==0){
      foundright = 1;
  }

  for(int k = 0; k < NMOTORS; k++){
    Serial.print(target[k]);
    Serial.print(" ");
    Serial.print(pos[k]);
    Serial.print(" ");
    
  }
  Serial.println();
  }
  posi[0] = 0;
  posi[1] = 0;
}

void readsensors(){
	for(int x = 0; x < N_SENS; x++){
    sens_scaled[x] = analogRead(SENSOR[x]) - sens_min[x];
    sens_scaled[x] *= R_SENS;
    sens_scaled[x] /= (sens_max[x] - sens_min[x]);
	}
}


void motorSpeed(int M1,int M2){
  int pwm1=map(abs(M1),0,1000,0,255);
  int pwm2=map(abs(M2),0,1000,0,255);
  ledcWrite(0,pwm1);
  ledcWrite(1,pwm2);
  digitalWrite(in1[0],(M1>=0) ? HIGH :LOW);
  digitalWrite(in2[0],(M1>=0) ? LOW :HIGH);
  digitalWrite(in1[1],(M2>=0) ? HIGH :LOW);
  digitalWrite(in2[1],(M2>=0) ? LOW :HIGH);
}


void calibrate(int cal_time, int cal_speed, int cal_dir){
  unsigned long ms = 0;
  ms = millis();
  digitalWrite(LEDR, LOW);
 	while((ms + cal_time) > millis()){
    digitalWrite(LEDR, millis()%100 < 50);
	    if(cal_dir == RIGHT_DIR)  {motorSpeed(-cal_speed, cal_speed);}
	    if(cal_dir == LEFT_DIR)  motorSpeed(cal_speed, -cal_speed);
	    int sens_value[N_SENS];
	    for(int x = 0; x < N_SENS; x++){
	      sens_value[x] = analogRead(SENSOR[x]);
	      sens_min[x] = (sens_value[x] < sens_min[x]) ? sens_value[x] : sens_min[x];
	      sens_max[x] = (sens_value[x] > sens_max[x]) ? sens_value[x] : sens_max[x];
	    	}
  	}
  motorSpeed(0, 0);
  digitalWrite(LEDR, LOW);
}


float get_PID_correction(float line, float last_line, float kp, float kd, float ki){
  float proportional = line;
  float derivative = line - last_line;
  float integral = line + last_line;
  float correction = ( proportional*KP + derivative*KD + KI* integral);
	return correction;
}


float get_line_pos(int color, int last_dir){
	float line = 0;
	int line_detected = 0;
	float avg_num = 0;          
	float avg_den = 0;          
	for(int x = 0; x < N_SENS; x++){
		readsensors();
	      	if(sens_scaled[x] >= (float) R_SENS * ((float)P_LINE_MIN / 100.0)){  
		    line_detected = 1;
		    }
		avg_num += sens_scaled[x] * x * WEIGHT_UNIT;
		avg_den += sens_scaled[x];        
		}
	if(line_detected == 1){
		line = avg_num / avg_den;                           
		line = line - (WEIGHT_UNIT * (N_SENS - 1) / 2);     
		}
	else{
		line = WEIGHT_UNIT * (N_SENS - 1) * last_dir;       
		line = line - (WEIGHT_UNIT * (N_SENS - 1) / 2);     
		}
	return line;
}



void race(void){
  while(1){
    last_line_pos = line_pos;
    line_pos = get_line_pos(COLOR, (last_line_pos>0));
    float PID_correction = get_PID_correction(line_pos, last_line_pos, KP, KD, KI);
    float max_correction = SPEED;                  
    if(PID_correction > 0){
      PID_correction = (PID_correction > max_correction)? max_correction : PID_correction;
      motorSpeed(SPEED, SPEED - PID_correction);
    	}	
    else{
      PID_correction = (PID_correction < -max_correction)? -max_correction : PID_correction;
      motorSpeed(SPEED + PID_correction, SPEED);
    	}
    if (sens_scaled[1]>600 && sens_scaled[2]>600 && sens_scaled[3]> 600 && sens_scaled[4]>600){
      return;
    }
    if(digitalRead(sensLeft)==0||digitalRead(sensRight)==0){
      return;
    }
  }
}


void simplify_path()
{// only simplify the path if the second-to-last turn was a 'B'
  if (path_length < 3 || path[path_length-2] != 'B')
    return;

  int total_angle = 0;
  int i;
  for (i = 1; i <= 3; i++)
  {
    switch (path[path_length - i])
    {
    case 'R':
      total_angle += 90;
      break;
    case 'L':
      total_angle += 270;
      break;
    case 'B':
      total_angle += 180;
      break;
    }
  }
  // Get the angle as a number between 0 and 360 degrees.
  total_angle = total_angle % 360;
  // Replace all of those turns with a single one.
  switch (total_angle)
  {
  case 0:
    path[path_length - 3] = 'S';
    break;
  case 90:
    path[path_length - 3] = 'R';
    break;
  case 180:
    path[path_length - 3] = 'B';
    break;
  case 270:
    path[path_length - 3] = 'L';
    break;
  }
  // The path is now two steps shorter.
  path_length -= 2;
}


unsigned char selectTurn(unsigned char foundLeft,unsigned char foundStraight,unsigned char foundRight){
  // returns values for turning
  if(foundLeft){
    return 'L';
  }
  if(foundStraight){
    return 'S';
  }
  if(foundRight){
    return 'R';
  }
  else
    return 'B';
}

void turni(unsigned char dir){
  switch (dir)
  {
  case 'R':
    attachInterrupt(digitalPinToInterrupt(enca[0]),readEncoder0,RISING);
    attachInterrupt(digitalPinToInterrupt(enca[1]),readEncoder1,RISING);
    set(120, 120, 250);
    set(-190, 190, 200);
    detachInterrupt(enca[0]);
    detachInterrupt(enca[1]);
    motorSpeed(0,0);
    break;
  case 'L':
    attachInterrupt(digitalPinToInterrupt(enca[0]),readEncoder0,RISING);
    attachInterrupt(digitalPinToInterrupt(enca[1]),readEncoder1,RISING);
    set(120, 120, 250);
    set(190, -190,200);
    detachInterrupt(enca[0]);
    detachInterrupt(enca[1]);
    break;
  case 'B':
    attachInterrupt(digitalPinToInterrupt(enca[0]),readEncoder0,RISING);
    attachInterrupt(digitalPinToInterrupt(enca[1]),readEncoder1,RISING);
    set(120, 120, 250);
    set(405, -405, 270);
    detachInterrupt(enca[0]);
    detachInterrupt(enca[1]);
    break;
  case 'S':
    attachInterrupt(digitalPinToInterrupt(enca[0]),readEncoder0,RISING);
    attachInterrupt(digitalPinToInterrupt(enca[1]),readEncoder1,RISING);
    set(120, 120, 50);
    detachInterrupt(enca[0]);
    detachInterrupt(enca[1]);
    break;
  }
}

void turn(unsigned char dir){
  switch (dir)
  {
  case 'R':
    attachInterrupt(digitalPinToInterrupt(enca[0]),readEncoder0,RISING);
    attachInterrupt(digitalPinToInterrupt(enca[1]),readEncoder1,RISING);
    set(-190, 190, 200);
    detachInterrupt(enca[0]);
    detachInterrupt(enca[1]);
    motorSpeed(0,0);
    break;
  case 'L':
    attachInterrupt(digitalPinToInterrupt(enca[0]),readEncoder0,RISING);
    attachInterrupt(digitalPinToInterrupt(enca[1]),readEncoder1,RISING);
    set(190, -190,200);
    detachInterrupt(enca[0]);
    detachInterrupt(enca[1]);
    break;
  case 'B':
    attachInterrupt(digitalPinToInterrupt(enca[0]),readEncoder0,RISING);
    attachInterrupt(digitalPinToInterrupt(enca[1]),readEncoder1,RISING);
    set(405, -405, 270);
    detachInterrupt(enca[0]);
    detachInterrupt(enca[1]);
    break;
  case 'S':
    break;
  }
}

void setup() {
  Serial.begin(115200);
  for(int k = 0; k < NMOTORS; k++){
    pinMode(enca[k],INPUT);
    pinMode(encb[k],INPUT);
    pinMode(pwm[k],OUTPUT);
    pinMode(in1[k],OUTPUT);
    pinMode(in2[k],OUTPUT);
    pinMode(led,OUTPUT);
    pid[k].setParams(3,.1,0.001,250);
  }
  pinMode(sensLeft,INPUT);
  pinMode(sensRight,INPUT);
  ledcSetup(0,30000,8);
  ledcSetup(1,30000,8);
  ledcAttachPin(pwm[0],0);
  ledcAttachPin(pwm[1],1);
  attachInterrupt(digitalPinToInterrupt(enca[0]),readEncoder0,RISING);
  attachInterrupt(digitalPinToInterrupt(enca[1]),readEncoder1,RISING);
  for(int x = 0; x < N_SENS; x++){
    	pinMode(SENSOR[x], INPUT);
  	  }
  for(int x = 0; x < N_SENS; x++){
      sens_max[x] = 0;
      sens_min[x] = 4000;
  	  }
  Serial.println("target pos");
  delay(2000);
}

void loop() { 
  detachInterrupt(enca[0]);
  detachInterrupt(enca[1]);
  calibrate(CAL_TIME, CAL_SPEED, RIGHT_DIR);
  calibrate(CAL_TIME, CAL_SPEED, LEFT_DIR);
  calibrate(CAL_TIME, CAL_SPEED, RIGHT_DIR);
  calibrate(CAL_TIME, CAL_SPEED, LEFT_DIR);
  digitalWrite(LEDR, HIGH);
  delay(5000);
  digitalWrite(LEDR, LOW);
  while(1){ 
    race();
    posi[0] = 0;
    posi[1] = 0;

    attachInterrupt(digitalPinToInterrupt(enca[0]),readEncoder0,RISING);
    attachInterrupt(digitalPinToInterrupt(enca[1]),readEncoder1,RISING);
    sets(120,120, 250);
    detachInterrupt(enca[0]);
    detachInterrupt(enca[1]);
    
    readsensors();
    if(sens_scaled[2]<500||sens_scaled[3]<500){
      foundstraight = 1;
    }
    readsensors();
    if((sens_scaled[0] < 200 && sens_scaled[1] < 200 && sens_scaled[2] < 200 && sens_scaled[3] < 200) ||(sens_scaled[2] < 200 && sens_scaled[3] < 200 && sens_scaled[4] <200 && sens_scaled[5] < 200) ){
      motorSpeed(0,0);
      break;
    }
    unsigned char dir = selectTurn(foundleft, foundstraight, foundright);
    foundleft = 0;
    foundright = 0;
    foundstraight = 0;
    turn(dir);
    path[path_length] = dir;
    path_length++;
    simplify_path();
    }
    while(1){
      digitalWrite(LEDR, HIGH);
      delay(10000);
      digitalWrite(LEDR, LOW);
      int i;
      for (i = 0; i < path_length; i++){
        race();
        turni(path[i]);
      }
      race();
      motorSpeed(-500,-500);
      delay(100);
      motorSpeed(0,0);
    }
}

