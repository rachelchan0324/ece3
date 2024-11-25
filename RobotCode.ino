#include <ECE3.h>

// SENSOR VARIABLES
uint16_t sensorValues[8];
int sensorMinimums[8] = {534, 558, 606, 582, 511, 558, 606, 630};
int sensorMaximums[8] = {2142, 2500, 2500, 2326, 1981, 2465, 2500, 2500};
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
const int base_speed = 37;
bool loop_done = false;
bool arch_done = false;
bool harpin_done = false;
bool turnaround_done = false;
int encoder_count = 0;

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

  resetEncoderCount_left();
  resetEncoderCount_right();
}

void loop() {
  float error = calculateError();

  if(error == 0 && !loop_done) // loop
  {
    loopTurn();
    return;
  }
  elif (error == 0 && loop_done && !turnaround_done) // turnaround 
  {
    turnaround();
    return; // needs to get new sensorValues
  }
  
  if(first_run) // relies only on kp
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
  encoder_count = int((abs(getEncoderCount_left()) + abs(getEncoderCount_right())) / 2.0); // update encoder counts
  prev_error = error; 
  first_run = false;
}

float calculateError()
{
  float error = 0;
  ECE3_read_IR(sensorValues);
  float normalizedValue;
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
  return error;
}

void turnaround()
{
  int starting_encoder_count = encoder_count;
  while(abs(encoder_count - starting_encoder_count) < 375) // 375 encoder counts for 180 turn
  {
    analogWrite(left_pwm_pin, base_speed);
    analogWrite(right_pwm_pin, base_speed);
    digitalWrite(left_dir_pin, HIGH); // one motor forward
    digitalWrite(right_dir_pin, LOW); // other motor backward
    encoder_count = int((abs(getEncoderCount_left()) + abs(getEncoderCount_right())) / 2.0);
  }
  digitalWrite(left_dir_pin, LOW); // back to forward
  turnaround_done = true;
}

void loopTurn()
{
  int starting_encoder_count = encoder_count;
  // move forward past the horizontal line
  while(abs(encoder_count - starting_encoder_count) < 125)
  {
    analogWrite(left_pwm_pin, base_speed);
    analogWrite(right_pwm_pin, base_speed);
    encoder_count = int((abs(getEncoderCount_left()) + abs(getEncoderCount_right())) / 2.0); // update encoder counts
  }

  starting_encoder_count = encoder_count;
  // perform hardcoded loop
  while(abs(encoder_count - starting_encoder_count) < 550)
  {
    analogWrite(left_pwm_pin, 125);
    analogWrite(right_pwm_pin, 0.1);
    delay(100);

    encoder_count = int((abs(getEncoderCount_left()) + abs(getEncoderCount_right())) / 2.0); // update encoder counts

    analogWrite(left_pwm_pin, 0);
    analogWrite(right_pwm_pin, 0);
    delay(100);  // Wait 1 second - adjust time as needed
  }
  prev_error = calculateError();
  loop_done = true;
}
