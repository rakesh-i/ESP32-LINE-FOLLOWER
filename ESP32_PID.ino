/* Motor pins */
#define MOT1 18
#define MOT2 5
#define MOT3 22
#define MOT4 23
#define PWM1 21
#define PWM2 19

#define LEDL 15 //Left led
#define LEDR 2  //Right led
#define LEDE 4  //End light
const int WHITE = 1;
const int BLACK  = 0;

#define COLOR   BLACK    
const int N_SENS  =      6 ;     //Number of sensors
#define R_SENS        1000   //Sensor readings are mapped to this range
#define WEIGHT_UNIT   1000    //Unit for weighted average
const int RIGHT_DIR =    1;
const int LEFT_DIR =    0;
#define CAL_SPEED     300  //Sensor calibration speed
#define CAL_TIME      880     //Calibration time
#define P_LINE_MIN    0.5     //Minimum brightness percentage to consider part of the line

float sens_scaled[6];
const float SPEED = 500;
const float KP = .7;
const float KD =1.4 ;
const float KI =.0001;
int sensRight = 14;
int sensLeft = 34;
unsigned long ms = 0;
const int SENSOR[N_SENS] = {27,26,25,33,32,35};     //Arduino pinsint sens_max[N_SENS];          //Maximum value each sensor measures (calibrated)
int sens_min[N_SENS]; 
int sens_max[8];//Minimum value each sensor measures (calibrated)
int start = 0;
float line_pos = 0;
float last_line_pos = 0;

void calibrate(int , int , int);
void motorSpeed(int ,int );
void setup(){
  Serial.begin(115200);
  pinMode(MOT1,OUTPUT);
  pinMode(MOT2,OUTPUT);
  pinMode(MOT3,OUTPUT);
  pinMode(MOT4,OUTPUT);
  pinMode(PWM1,OUTPUT);
  pinMode(PWM2,OUTPUT);
  pinMode(LEDL,OUTPUT);
  pinMode(LEDR,OUTPUT);
  pinMode(LEDE,OUTPUT);
  pinMode(sensLeft,INPUT);
  pinMode(sensRight,INPUT);
  ledcSetup(0,30000,8);
  ledcSetup(1,30000,8);

  ledcAttachPin(PWM1,0);
  ledcAttachPin(PWM2,1);

  for(int x = 0; x <= N_SENS; x++){
    pinMode(SENSOR[x], INPUT);
  }

  for(int x = 0; x < N_SENS; x++){
      sens_max[x] = 0;
      sens_min[x] = 4000;
  }
}


void loop(){
 
  
      
      calibrate(CAL_TIME, CAL_SPEED, RIGHT_DIR);
      calibrate(CAL_TIME, CAL_SPEED, LEFT_DIR);
      calibrate(CAL_TIME, CAL_SPEED, RIGHT_DIR);
      calibrate(CAL_TIME, CAL_SPEED, LEFT_DIR);
      delay(2000);
  
  
             while(1){ race();
           
             
 /*Serial.print(sens_scaled[0]);
      Serial.print(" ");
      Serial.print(sens_scaled[1]);
      Serial.print(" ");
      Serial.print(sens_scaled[2]);
      Serial.print(" ");
      Serial.print(sens_scaled[3]);
      Serial.print(" ");
      Serial.print(sens_scaled[4]);
      Serial.print(" ");
      Serial.print(sens_scaled[5]);
      Serial.print(" ");
      Serial.print(digitalRead(sensLeft));
       Serial.print(" ");
          Serial.print(digitalRead(sensRight));
       Serial.print(" ");
      Serial.print(line_pos);
      Serial.println();
      
      Serial.print(" ");
      Serial.println(sens_scaled[6]);
      Serial.print(" ");
      Serial.print(sens_scaled[7]);
      Serial.print("||");
      Serial.print(sens_min[0]);
      Serial.print(" ");
      Serial.print(sens_max[0]);
      Serial.print(" ");
      Serial.print(line_pos);
      Serial.println();*/
      
      
      }
  
 
  
  
}


void race(void){
    last_line_pos = line_pos;
    line_pos = get_line_pos(COLOR, (last_line_pos>0));
        
    float PID_correction = get_PID_correction(line_pos, last_line_pos, KP, KD, KI);
    float max_correction = SPEED;                   //Can be changed to a lower value in order to limit the correction, needs to be at most SPEED
    
    if(PID_correction > 0){
        PID_correction = (PID_correction > max_correction)? max_correction : PID_correction;
        motorSpeed(SPEED, SPEED - PID_correction);
    }else{
        PID_correction = (PID_correction < -max_correction)? -max_correction : PID_correction;
        motorSpeed(SPEED + PID_correction, SPEED);
    }
   
}


void motorSpeed(int M1,int M2){
  int pwm1=map(abs(M1),0,1000,0,255);
  int pwm2=map(abs(M2),0,1000,0,255);
  ledcWrite(0,pwm1);
  ledcWrite(1,pwm2);
  digitalWrite(MOT1,(M1>=0) ? HIGH :LOW);
  digitalWrite(MOT2,(M1>=0) ? LOW :HIGH);
  digitalWrite(MOT3,(M2>=0) ? HIGH :LOW);
  digitalWrite(MOT4,(M2>=0) ? LOW :HIGH);
  
}


void calibrate(int cal_time, int cal_speed, int cal_dir){
  ms = millis();
  digitalWrite(LEDL, LOW);
  digitalWrite(LEDR, LOW);
  while((ms + cal_time) > millis()){
    digitalWrite(LEDL, millis()%100 < 50);
    digitalWrite(LEDR, millis()%100 < 50);
    digitalWrite(LEDE,millis()%100<50);//Blink led
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
  digitalWrite(LEDL, HIGH);
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
    
    float avg_num = 0;          //Average numerator
    float avg_den = 0;          //Average denominator
    
    for(int x = 0; x < N_SENS; x++){
        //Scale from 0 to R_SENS
  
        readsensors();
       
        if(sens_scaled[x] >= (float) R_SENS * ((float)P_LINE_MIN / 100.0)){   //At least one sensor has to detect a line
            line_detected = 1;
        }
        
        avg_num += sens_scaled[x] * x * WEIGHT_UNIT;
        avg_den += sens_scaled[x];        
    }
    if(line_detected == 1){
        line = avg_num / avg_den;                           //Weighted average
        line = line - (WEIGHT_UNIT * (N_SENS - 1) / 2);     //Change scale from 0 _ 7000 to -3500 _ 3500
        digitalWrite(LEDL, LOW);
    }else{
        line = WEIGHT_UNIT * (N_SENS - 1) * last_dir;       //Use last direction to calculate error as the maximum value
        line = line - (WEIGHT_UNIT * (N_SENS - 1) / 2);     //Change scale
        digitalWrite(LEDL, HIGH);
    }
    
    return line;
}


void readsensors(){
for(int x = 0; x < N_SENS; x++){
        //Scale from 0 to R_SENS
        sens_scaled[x] = analogRead(SENSOR[x]) - sens_min[x];
        sens_scaled[x] *= R_SENS;
        sens_scaled[x] /= (sens_max[x] - sens_min[x]);
}
}
