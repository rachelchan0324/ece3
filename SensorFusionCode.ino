#include <ECE3.h>

uint16_t sensorValues[8];
uint16_t sensorMinimums[8] = {748, 652, 627, 627, 652, 604, 627, 676};
// uint16_t sensorMinimums[8]= {435, 389, 435, 435, 412, 458, 480, 642};
uint16_t sensorMaximums[8] = {1611, 1872, 2465, 1920, 1398, 1943, 2180, 2299};
int32_t sensorWeights[8] = {-8, -4, -2, -1, 1, 2, 4, 8};

void setup() {
  ECE3_Init();
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission
  delay(1000);
}

void loop() {
  ECE3_read_IR(sensorValues);
  int32_t error = 0;
  int normalizedValue;

  for (unsigned char i = 0; i < 8; i++) {
    // normalization of sensorValues
    normalizedValue = sensorValues[i];
    normalizedValue -= sensorMinimums[i];
    normalizedValue *= 1000;
    normalizedValue /= (sensorMaximums[i] - sensorMinimums[i]);

    error += (normalizedValue * sensorWeights[i]);
  }
  error /= 4;
  Serial.print(error);
  Serial.println();
  delay(1500);
}
