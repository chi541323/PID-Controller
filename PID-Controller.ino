
/* upper right => M1, right => M2, lower right => M3
   lower left => M4,  left => M5,  upper left => M6   */


#include <Encoder.h>
#include <PID_v1.h>


#define M1_H 0 // Motor1 Horizontal
#define M1_V 1 // Motor1 Vertical
#define M2_H 2 // Motor2 Horizontal
#define M2_V 3 // Motor2 Vertical
#define M3_H 4 // Motor3 Horizontal
#define M3_V 5 // Motor3 Vertical

// Encoder variable
Encoder ENC[6] = {Encoder(18, 50),  // M1_H
                  Encoder(19, 51),  // M1_V
                  Encoder(20, 52),  // M2_H
                  Encoder(21, 53),  // M2_V
                  Encoder(2, 48),   // M3_H
                  Encoder(3, 49)    // M3_V
                 };
long nowPos[6];
long oldPos[6] = { -999, -999, -999, -999, -999, -999};


// PID variable
double Setpoint[6] = {0, 0, 0, 0, 0, 0};
double Input[6] = {0, 0, 0, 0, 0, 0};
double Output[6] = {0, 0, 0, 0, 0, 0};
double kp = 2, ki = 0.00001, kd = 0;


// input: current encoder value ; output: output voltage; Setpoint: target encoder value
PID pid[6] = {PID(&Input[M1_H], &Output[M1_H], &Setpoint[M1_H], kp, ki, kd, DIRECT),
              PID(&Input[M1_V], &Output[M1_V], &Setpoint[M1_V], kp, ki, kd, DIRECT),
              PID(&Input[M2_H], &Output[M2_H], &Setpoint[M2_H], kp, ki, kd, DIRECT),
              PID(&Input[M2_V], &Output[M2_V], &Setpoint[M2_V], kp, ki, kd, DIRECT),
              PID(&Input[M3_H], &Output[M3_H], &Setpoint[M3_H], kp, ki, kd, DIRECT),
              PID(&Input[M3_V], &Output[M3_V], &Setpoint[M3_V], kp, ki, kd, DIRECT)
             };


// MOTOR Pin
const int STBY[3] = {26, 27, 36}; //standby, {driver1 driver2,driver3}
const int PWM[6] = {4, 5, 6, 7, 8, 9}; //{D1_A,D1_B,D2_A,D2_B,D3_A,D3_B}
const int IN1[6] = {22, 24, 28, 30, 32, 34}; // {D1_A,D1_B,D2_A,D2_B,D3_A,D3_B}
const int IN2[6] = {23, 25, 29, 31, 33, 35}; // {D1_A,D1_B,D2_A,D2_B,D3_A,D3_B}

int target[6] = {0, 0, 0, 0, 0, 0};
const int encoderValue = 89; // encoder value per 1mm

// encoder value acceptable range
const int acceptableOffset = 1;

void setup() {

  Serial.begin(115200);

  for (int i = 0; i < 3; i++) {
    pinMode(STBY[i], OUTPUT);
  }

  for (int i = 0; i < 6; i++) {
    pinMode(PWM[i], OUTPUT);
    pinMode(IN1[i], OUTPUT);
    pinMode(IN2[i], OUTPUT);
    pid[i].SetMode(AUTOMATIC);
    pid[i].SetSampleTime(1);
    // PWM -255~255
    pid[i].SetOutputLimits(-255, 255);
  }

}

void loop() {

  readEncoder();
  pidCompute();
  motorStandby();

}


void readEncoder() {

  for (int i = 0; i < 6; i++) {
    nowPos[i] = ENC[i].read();
    if (nowPos[i] != oldPos[i]) {
      oldPos[i] = nowPos[i];
      Input[i] = nowPos[i];
      Serial.print("encoder:");
      Serial.print(i);
      Serial.print(" val: ");
      Serial.println(nowPos[i]);
    }
  }
}

void pidCompute() {
  for (int i = 0; i < 6; i++) {
    if (abs(Setpoint[i] - nowPos[i]) > acceptableOffset) {
      pid[i].Compute();
      pwmOut(i);
    }
    else{
      Output[i] = 0;
    }
  }
}


void pwmOut(int x) {
  // Serial.println(Output[3]);
  digitalWrite(STBY[x / 2], HIGH);
  if (Output[x] > 0) {
    digitalWrite(IN1[x], HIGH);
    digitalWrite(IN2[x], LOW);
  }
  else {
    digitalWrite(IN1[x], LOW);
    digitalWrite(IN2[x], HIGH);
  }
  analogWrite(PWM[x], abs(Output[x]));
}

//called when data from serial port is available
void serialEvent() {
  // data format -> motor number, y-axis value, x-axis value
  String incomingStr = Serial.readStringUntil('\n');
  int motorNum = getValue(incomingStr, ',', 0).toInt();
  float vDistance = getValue(incomingStr, ',', 1).toFloat();
  float hDistance = getValue(incomingStr, ',', 2).toFloat();
  targetCount(motorNum, vDistance, hDistance);

}

// calculate the target value
void targetCount(int motorNum, float vDistance, float hDistance) {
  Setpoint[motorNum * 2 - 2] = hDistance * encoderValue;
  Setpoint[motorNum * 2 - 1] = vDistance * encoderValue;
}


String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }

  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void motorStandby() {
  for (int i = 0 ; i < 5;  i += 2) {
    if (Output[i] == 0 && Output[i + 1] == 0) {
      digitalWrite(STBY[i / 2], LOW);
    }
  }
}
