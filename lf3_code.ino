#include <SparkFun_TB6612.h>

#define AIN1 4
#define BIN1 6
#define AIN2 3
#define BIN2 7
#define PWMA 9
#define PWMB 10
#define STBY 5 

const int offsetA = 1;
const int offsetB = 1;

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

const int lineFollowSensor0 = A0;
const int lineFollowSensor1 = A1;
const int lineFollowSensor2 = A2;
const int lineFollowSensor3 = A3;
const int lineFollowSensor4 = A4;

int LFSensor[5]={0, 0, 0, 0, 0};

float Kp = 1.0;  // Proportional gain
float Ki = 0.0;  // Integral gain
float Kd = 0.0;  // Derivative gain

int lsp, rsp;
int lfSpeed = 120;
int currentSpeed = 90;

// PID control variables

float previousError = 0;
float integral = 0;
// Function to compute PID output

float computePID(float error) 
  {
  // Proportional term
  float proportional = Kp * error;

  // Integral term
  integral += error;
  float integralTerm = Ki * integral;

  // Derivative term
  float derivative = error - previousError;
  float derivativeTerm = Kd * derivative;

  // PID output
  float output = proportional + integralTerm + derivativeTerm;

  // Save the current error as previous error for the next iteration
  previousError = error;

  return output;
}






void setup() {
  Serial.begin(9600);
  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
  pinMode(13, OUTPUT);

}

void loop() {


  while (digitalRead(11)) {}
  delay(1000);
  
  int sensorValue = getSensorValue(); 
  Serial.println(LFSensor[0]);
  Serial.println(LFSensor[1]);
  Serial.println(LFSensor[2]);
  Serial.println(LFSensor[3]);
  Serial.println(LFSensor[4]);
  int error = sensorValue;
  float PIDvalue = computePID(error);
  
  lsp = currentSpeed - PIDvalue;
  rsp = currentSpeed + PIDvalue;
  
  
  if (lsp > 255) {
    lsp = 255;
  }
  if (lsp < 0) {
    lsp = 0;
  }
  if (rsp > 255) {
    rsp = 255;
  }
  if (rsp < 0) {
    rsp = 0;
  }
  motor1.drive(lsp);
  motor2.drive(rsp); 
}

int getSensorValue() {

int error;

LFSensor[0] = digitalRead(lineFollowSensor0);
LFSensor[1] = digitalRead(lineFollowSensor1);
LFSensor[2] = digitalRead(lineFollowSensor2);
LFSensor[3] = digitalRead(lineFollowSensor3);
LFSensor[4] = digitalRead(lineFollowSensor4);



if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 1 )) error = 40;

else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 1 )) error = 30; 

else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 0 )) error = 20;

else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 0 )) error = 10;

else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 )) error = 0;

else if((LFSensor[0]== 0 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 )) error =- 10;

else if((LFSensor[0]== 0 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 )) error = -20;

else if((LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 )) error = -30;

else if((LFSensor[0]== 1 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 )) error = -40;

return error;

}









