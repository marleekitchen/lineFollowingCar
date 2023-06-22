#include <ECE3.h>

#define N_SENS 8
#define R_SENS 1000    //Sensor readings are mapped to this range
#define MOTORSPEED 60

const int left_nslp_pin=31; // nslp ==> awake & ready for PWM
const int left_dir_pin=29;
const int left_pwm_pin=40;
const int right_nslp_pin=11; // nslp ==> awake & ready for PWM
const int right_dir_pin=30;
const int right_pwm_pin=39;
const int LED_RF = 41;

uint16_t sensorValues[8]; // right -> left, 0 -> 7

long maxOfSumOfSensors = 12540;
int sens_max[N_SENS] = {1687, 1637, 1694, 1359.2, 1376, 1468, 1530.8, 1788};
int sens_min[N_SENS] = {805, 735, 806, 656.8, 712, 643, 675.2, 712};
int weight_val[N_SENS] = {-8,-4,-2,-1,1,2,4,8};
const float kp = 0.038;
const float kd = 0.35;
 
int sensorRPID = 0;
float line_pos = 0;
float prev_line_pos = 0;

void setup() {
  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);

  pinMode(right_nslp_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);

  digitalWrite(left_dir_pin,LOW);
  digitalWrite(right_dir_pin,LOW);
  
  digitalWrite(left_nslp_pin,HIGH);
  digitalWrite(right_nslp_pin,HIGH);

  ECE3_Init();
  delay(2000);
}

float get_line_pos(int last_dir){
    float sens_scaled[N_SENS];
    float line = 0;
    int line_detected = 0;
    float avg_num = 0;          
    float avg_den = 0;          

    for(int x = 0; x < N_SENS; x++){
        //Scale from 0 to R_SENS
        sens_scaled[x] = sensorValues[x] - sens_min[x];
        sens_scaled[x] *= R_SENS;
        sens_scaled[x] /= sens_max[x];
        sens_scaled[x]  *= weight_val[x];
   
        avg_num += sens_scaled[x];
        avg_den += sens_scaled[x];        
    }

    line = avg_num / 4;
      
    return line;
}

float get_PID_correction(float line, float last_line, float kp, float kd){
  float proportional = line;
  float derivative = (line - last_line);
  float correction = (kp * proportional + kd * derivative);

  return correction;
}

static int x = 0;
unsigned long startS = 0;
unsigned long endS = 0;

void loop() {
  ECE3_read_IR(sensorValues);

  int sumSensors = 0;
  for(int i = 0; i < N_SENS; ++i)
    sumSensors += sensorValues[i];

  endS = millis();
  if (sumSensors >= (maxOfSumOfSensors - 100))
  {
    switch (x) {
      case 0:
        {
        startS = millis();
        analogWrite(right_pwm_pin, 150);
        analogWrite(left_pwm_pin, 150);
        digitalWrite(left_dir_pin,HIGH);
        delay(400);
        digitalWrite(left_dir_pin,LOW);
        analogWrite(left_pwm_pin, MOTORSPEED);
        analogWrite(right_pwm_pin, MOTORSPEED);
        ++x;
        break;
        }
      case 1:
        if(endS - startS >= 600){
          analogWrite(left_pwm_pin, 0);
          analogWrite(right_pwm_pin, 0);
          digitalWrite(left_nslp_pin,LOW);
          digitalWrite(right_nslp_pin,LOW);
          }
        break;
    }
  }
   
  prev_line_pos = line_pos;
  line_pos = get_line_pos(prev_line_pos>0);

  float PID_CORR = get_PID_correction(line_pos, prev_line_pos, kp, kd);
    
    if(PID_CORR > 0){
        analogWrite(left_pwm_pin, MOTORSPEED +5);
        analogWrite(right_pwm_pin, MOTORSPEED + PID_CORR);
    }
    else{
        analogWrite(left_pwm_pin, MOTORSPEED + 5 - PID_CORR);
        analogWrite(right_pwm_pin, MOTORSPEED);
    }
  }
