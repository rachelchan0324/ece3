#include <ECE3.h>

// SENSOR VARIABLES
uint16_t sensorValues[8];
// uint16_t sensorMinimums[8] = {748, 652, 627, 627, 652, 604, 627, 676}; // sophia's bot
// uint16_t sensorMinimums[8]= {435, 389, 435, 435, 412, 458, 480, 642}; // rachel's bot (OG)
int sensorMinimums[8] = {534, 558, 606, 582, 511, 558, 606, 630}; // rachel's bot - recalibrated
// int sensorMinimums[8] = {771, 723, 675, 651, 651, 627, 627, 675 }; // sophia's bot - recalibrated
// uint16_t sensorMaximums[8] = {1611, 1872, 2465, 1920, 1398, 1943, 2180, 2299}; // rachel's bot (OG)
int sensorMaximums[8] = {2142, 2500, 2500, 2326, 1981, 2465, 2500, 2500}; // rachel's bot - recalibrated
// uint16_t sensorMaximums[8] = {2500, 2500, 2500, 2500, 2500}; // sophia's bot - recalibrated
int sensorWeights[8] = {-15, -14, -12, -8, 8, 12, 14, 15};

// PIN VARIABLES
const int left_nslp_pin=31; // nslp ==> awake & ready for PWM
const int left_dir_pin=29;
const int left_pwm_pin=40;
const int LED_RF = 41;
const int right_dir_pin = 30;
const int right_nslp_pin = 11;
const int right_pwm_pin = 39;

// CODE VARIABLES
float kp = 0.04;
float kd = 0.05;
float prev_error;
int base_speed = 37;
bool first_run = true;

void setup() {
  ECE3_Init();
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission
  delay(1000);

  // PIN SETUP
  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);
  digitalWrite(left_dir_pin,LOW);
  digitalWrite(left_nslp_pin, HIGH);
  pinMode(right_nslp_pin, OUTPUT);
  pinMode(right_dir_pin, OUTPUT);
  pinMode(right_pwm_pin, OUTPUT);
  digitalWrite(right_dir_pin, LOW);
  digitalWrite(right_nslp_pin, HIGH);
  pinMode(LED_RF, OUTPUT);
}

void loop() {
  ECE3_read_IR(sensorValues);
  
  float error = 0;
  float normalizedValue;

  // calculate error
  for (int i = 0; i < 8; i++) {
    // normalization of sensorValues
    if (sensorValues[i] < sensorMinimums[i])
      sensorMinimums[i] = sensorValues[i];
    if (sensorValues[i] > sensorMaximums[i])
      sensorMaximums[i] = sensorValues[i];

    normalizedValue = sensorValues[i];
    normalizedValue -= sensorMinimums[i];
    normalizedValue *= 1000;
    normalizedValue /= (sensorMaximums[i] - sensorMinimums[i]);
    error += (normalizedValue * sensorWeights[i]);
  }
  error /= 24;
 
  if (error == 0) // 
  {
      int left_encoder_count = 0;
      int right_encoder_count = 0;
      resetEncoderCount_left();
      resetEncoderCount_right();

      while(abs(left_encoder_count) < 375)
      {
        analogWrite(left_pwm_pin, base_speed);
        analogWrite(right_pwm_pin, base_speed);
        digitalWrite(left_dir_pin, HIGH); // one motor forward
        digitalWrite(right_dir_pin, LOW); // other motor backward
        left_encoder_count = getEncoderCount_left();
        right_encoder_count = getEncoderCount_right();
      }
  
      digitalWrite(left_dir_pin, LOW); // back to forward
      return;
  }
  
  if(first_run)
  {
      float proportional_term = error * kp;
      analogWrite(left_pwm_pin, base_speed - proportional_term);
      analogWrite(right_pwm_pin, base_speed + proportional_term);
  }
  else
  {
    float proportional_term = error * kp;
    float derivative = error - prev_error;
    float derivative_term = derivative * kd;

    analogWrite(left_pwm_pin, base_speed - proportional_term - derivative_term);
    analogWrite(right_pwm_pin, base_speed + proportional_term + derivative_term);
  }

  prev_error = error;
  first_run = false;
}
