#include <Encoder.h>
#include <PID_v1.h>

#define CHANGING_SPEED 0
#define CHANGING_ANGLE 1
#define NOTHING 2


//MOTOR
const int STBY = 11;
const int PWM = 6;
const int IN1 = 8;
const int IN2 = 7;

//Encoder
Encoder ENC(12, 13);
long oldPosition = 0;
long homePosition = 0;

//PID
double kp = 0;
double ki = 0;
double kd = 0;

double input = 0;
double output = 0;
double setpoint = 0;
double target = 0;

PID pid(&input, &output, &setpoint, kp, ki, kd, DIRECT);

//other
const int pidcount = 2500;
const int sampleRate = 20;
const long GERATIO = 298 * 12; //150 (motor gear ratio) * 12 (encoder)

unsigned long oldTime = 0;
int mode = CHANGING_SPEED;
int count = 0;
char cmd = ' ';


void setup() {
  Serial.begin(115200);

  pinMode(STBY, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(1);
  pid.SetOutputLimits(-255, 255);


}

void loop() {

  if (mode == CHANGING_SPEED) computeSpeed();
  if (mode == CHANGING_ANGLE) computeAngle();
  readSerial();
  pwmOut();

}

void readSerial() {

  if (Serial.available() > 0) {
    int incomingByte = Serial.read();
    bool sendBack = false;
    float in;
    while (incomingByte != '\n') {
      Serial.println(incomingByte);
      if (isLetter(incomingByte)) {
        cmd = incomingByte;
      }
      Serial.println(cmd);
      incomingByte = Serial.read();
      switch (cmd) {

        //PID
        case 'P': //setup pid p value (+)
          kp = myParseNumber(&incomingByte, true);
          pid.SetTunings(kp, ki, kd);

          break;

        case 'D': //setup pid d value (+)
          kd = myParseNumber(&incomingByte, true);
          pid.SetTunings(kp, ki, kd);

          break;

        case 'I': //setup pid i value (+)
          ki = myParseNumber(&incomingByte, true);
          pid.SetTunings(kp, ki, kd);

          break;

        //ACTUATE MOTOR
        case 'S': //speed in rpm (+)
          in = myParseNumber(&incomingByte, true);
          oldPosition = ENC.read();
          oldTime = millis();
          target = in;
          count = -1;
          mode = CHANGING_SPEED;
          
          break;

        case 'A': // rotate to angle relative to home (+)

          in = myParseNumber(&incomingByte, true);
          long nowPosition, offPosition;
          nowPosition = ENC.read();
          offPosition = angleToCount(in) - ((nowPosition - homePosition) % GERATIO);
          target = nowPosition + offPosition;
          count = 0;
          mode = CHANGING_ANGLE;
          break;

        case 'H': // set now angle as home 0 (x)
          homePosition = ENC.read();
          break;

      }

    }

    Serial.flush();

    if (sendBack) Serial.println("k");
  }

}

void pwmOut() {
  digitalWrite(STBY, HIGH);
  if (output > 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  else {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  analogWrite(PWM, abs(output));
}

void computeSpeed() {

  unsigned long newTime = millis();
  if (newTime - oldTime >= sampleRate && target != 0) {
    long newPosition = ENC.read();

    input = ((newPosition - oldPosition) * 1000 / ((newTime - oldTime) * GERATIO / 60.0));

    oldPosition = newPosition;
    oldTime = newTime;
  } else if (target == 0) {
    input = 0;
  }
  pidCompute(false);

}

void computeAngle() {
  input = ENC.read();
  if (count > -1) pidCompute(true);
}

void pidCompute(bool c) {
  //Serial.println("input: " + String(input) + ", output: " + String(output) + ", target: " + String(target));

  setpoint = target;
  while (!pid.Compute());

  if (c) { //prevent continuous rotate
    count++;
    if (count >= pidcount) {
      count = -1;
      output = 0;
    }
  }
}

// Handle Read Number

float myParseNumber(int *incomingByte, bool isFloat) {
  float in = 0;
  bool isRead = false;
  char inData[32];
  int index = 0;

  while (*incomingByte != '\n') {
    if (*incomingByte == char(-1)) { //if read too fast...
      *incomingByte = Serial.read();
      continue;
    }
    if (!isNumber(*incomingByte, isFloat)) break;

    isRead = true;
    inData[index] = *incomingByte;
    index++;
    *incomingByte = Serial.read();
  }
  inData[index] = '\0';

  delay(10);

  if (isRead) {
    if (isLetter(*incomingByte)) {
      cmd = *incomingByte;
      isRead = false;
    } else cmd = '\0';

    char data_char[index + 1];

    for (int i = 0; i < index; i++) {
      data_char[i] = inData[i];
    }

    in = atof(data_char);
    isRead = false;
    index = 0;
  }
  return in;
}

bool isNumber(char c, bool isFloat) {
  return isFloat ? isDigit(c) || (c == '.') || (c == '-') : isDigit(c) || (c == '-') ;
}

bool isLetter(char c) {
  return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z');
}

long angleToCount(double angle) {
  return (angle * GERATIO) / 360;
}
