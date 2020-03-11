#include <Encoder.h>
#include <PID_v1.h>

#define CLOCKWISE 1
#define C_CLOCKWISE -1
#define A 0 //A is upper motor
#define B 1 //B is lower motor
#define SIN 0
#define TRI 1
#define SININT 2


//MOTOR
const int STBY = 11; //standby
const int PWM[2] = {6, 5};
const int IN1[2] = {8, 10};
const int IN2[2] = {7, 9};

//Encoder A
Encoder ENC[2] = {Encoder(12,13), Encoder(3, 4)};

const int BTN[2] = {25, 24};
int btnState[2] = {0, 0};

double kp[2] = {5, 3}; //5
double ki[2] = {20, 0}; //20
double kd[2] = {0, 0}; //0
int pidcount = 2500;

double input[2] = {0, 0};
double output[2] = {0, 0};
double setpoint[2] = {0, 0};
bool counting[2] = {false, false};
long homing[2] = {0, 0};

double testSpeed = 255;
double prevSpeed = 128;

PID pid[2] = {PID(&input[A], &output[A], &setpoint[A], kp[A], ki[A], kd[A], DIRECT), 
              PID(&input[B], &output[B], &setpoint[B], kp[B], ki[B], kd[B], DIRECT)};

double target[2] = {0, 0};
int count[2] = {0, 0};

int okTimeout = -1;

int timeFactor = 64;
int nowMotor = A;

char inData[32];
bool isRead = false;
int index = 0;
char cmd;
bool connectUnity = false;
bool connectProcessing = false;

long oldPosition = 0;
unsigned long startTime;
unsigned long newTime;
unsigned long oldTime = 0;

int sampleRate = 20;
double freq = 0;
double amplitude = 0;
double velocity = 0;
int wave = SIN;
bool lastB = false;
bool checkEncoder = false;


long GRATE = 1200; //30(motor gear ratio)* 12(encoder) * 60/18(device gear ratio)
double GRATE_S = 297.92 * 12; //297.92(motor gear ratio)* 12(encoder); mm
double SRATE = 25.4*PI;

bool closestAngle = false;

double freqIntegral = 0;
double bumpFreq = 0;
double bumpAmplitude = 0;
double bumpTime = 0;
bool bumping = false;

//test
void setup(){
  
  Serial.begin(115200);
  //TCCR1B = TCCR1B & 0b11111000 | 0x01; // set 31Kh PWM
  
  pinMode(STBY, OUTPUT);

  for (int i=0; i<2; i++) {
    pinMode(PWM[i], OUTPUT);
    pinMode(IN1[i], OUTPUT);
    pinMode(IN2[i], OUTPUT);
    
    pid[i].SetMode(AUTOMATIC);
    pid[i].SetSampleTime(1);
    pid[i].SetOutputLimits(-255,255);
  }

}

void loop(){

  if (connectUnity || connectProcessing) {
    //handle pid compute
    //if (counting[A]) 

    if (counting[A] || connectProcessing) computeSpeed(A);
    if (counting[B]) computeAngle(B);
  
    okTimeout++;
  
    if ((okTimeout%5)==0) { //change to every loop?
      readSerial();
    }
  
    if (okTimeout > 1000 && connectUnity) { //change?
      
      Serial.println("k");
      okTimeout = 0;
    }
  
    //handle btn state
    //handleBtn();
    
    //move motor
    pwmOut();
  } else {
    readSerial();
    if (checkEncoder) {
      Serial.print("Enc 0: "+String(ENC[0].read())+", Enc 1: "+String(ENC[1].read())+"\n");
    }
  }

}

void computeSpeed(int i) {
  if (millis() - oldTime >= sampleRate) {
    long newPosition = ENC[i].read();
    newTime = millis();
    input[i] = (newPosition-oldPosition)*SRATE*1000 / ((newTime-oldTime)*GRATE_S); //mm/s
    calculateTarget(i, newTime);
    
    if (connectProcessing) {
      Serial.print(int16_t(input[i]));
      //Serial.print(")");
      //Serial.print(int16_t(3));
      Serial.print("\r");
    } else {
      Serial.print("new-old pos = ");
      Serial.print(newPosition-oldPosition);
      Serial.print(", new-old time = ");
      Serial.print(newTime-oldTime);
      Serial.print (",speed = ");
      Serial.println (input[i]);
    }
    
    oldPosition = newPosition;
    oldTime = newTime;
  }
  if (counting[i]) pidCompute(i, false);
}

void calculateTarget(int i, unsigned long newTime) {
  double out;
  if (wave == SIN) {//sin 
    out = velocity + amplitude*sin((newTime-startTime)*2*PI*freq/1000); //second
  } else if (wave == TRI) {
    double temp;
    unsigned long t = ((newTime-startTime)/1000) % 4;
    if (t > 3) {
      temp = t-4;
    } else if (t > 1) {
      temp = 2-t;
    } else {
      temp = t;
    }
    out = velocity + amplitude*temp*freq/4;
  } else if (wave == SININT) {
    freqIntegral += (newTime-oldTime)*freq/1000;
    out = velocity + amplitude*sin(2*PI*freqIntegral);
    
  }
  
  if (bumping) {
    double sign = velocity >= 0 ? 1 : -1;
    out = velocity + sign * bumpAmplitude*sin((newTime-bumpTime)*2*PI*bumpFreq/1000);
    if ((newTime-bumpTime)*bumpFreq/1000 >=  1) bumping = false;
  }

  target[i] = out;
  
}

void computeAngle(int i) {
  long newPosition = ENC[i].read();
  input[i] = newPosition;
  pidCompute(i, true);
}

void pidCompute(int i, bool c) {  
  setpoint[i] = target[i];
  while(!pid[i].Compute());
  //Serial.println("o = "+String(output[i]));
  if(c) {
    count[i]++;
    if(count[i] >= pidcount) {
      counting[i] = false;
      output[i] = 0;
    }
  }
}

void handleBtn() {
  
  //A btn 
  int tempBtnState = btnState[A];
  btnState[A] = digitalRead(BTN[A]);
  if (btnState[A] == HIGH && tempBtnState == LOW) output[A] = testSpeed;
  else if (btnState[A] == LOW && tempBtnState == HIGH) output[A] = 0;

  //B btn
  tempBtnState = btnState[B];
  btnState[B] = digitalRead(BTN[B]);
  if (btnState[B] == HIGH && tempBtnState == LOW){
    //Serial.println(ENC[B].read());
    target[B] = ENC[B].read() + angleToCount(90)*CLOCKWISE;
    counting[B] = true;
    count[B] = 0;
//      int tempSpeed = 0;
//      tempSpeed = testSpeed;
//      testSpeed = prevSpeed;
//      prevSpeed = tempSpeed;
  }

}

void readSerial() {
  int ini = 0;
  float inf = 0;
  float in = 0;
  long temp = 0;
  long temp2 = 0;
  if (lastB && counting[A] == 0 && counting[B] == 0) {
    connectUnity = false;
    connectProcessing = false;
    lastB = false;
  }
  if (Serial.available() > 0) {
    int incomingByte = Serial.read();
    bool sendback = false;
    while (incomingByte != '\n') { //&& (isLetter(incomingByte) || isNumber(incomingByte, true) || incomingByte == char(-1)) 
      //Serial.print(">"+String(incomingByte)+"\n");
      sendback = true;
      if (isLetter(incomingByte)) {
        cmd = incomingByte;
        incomingByte = Serial.read();
        
        isRead = false;
      } else { //if not read
         incomingByte = Serial.read();
      }
      
      switch(cmd) {
        /*
          case 'i':
            ini = int(myParseNumber(incomingByte, false));
            
            Serial.print(toClosest(ini, 360));
            break;
          case 'f':
            inf = myParseNumber(incomingByte, true);
            Serial.print("inf:"+String(inf));
            break;*/
          case 'M': //which motor
            nowMotor = int(myParseNumber(&incomingByte, false));
            //Serial.print(nowMotor);
            if (nowMotor != A && nowMotor != B) nowMotor = -1;
            break;
            
          case 'P': //setup pid p value
            kp[nowMotor]=myParseNumber(&incomingByte, true);
            pid[nowMotor].SetTunings(kp[nowMotor],ki[nowMotor],kd[nowMotor]);
            
            break;
            
          case 'D': //setup pid d value
            kd[nowMotor]=myParseNumber(&incomingByte, true); 
            pid[nowMotor].SetTunings(kp[nowMotor],ki[nowMotor],kd[nowMotor]);
            
            break;
            
          case 'I': //setup pid i value
            ki[nowMotor]=myParseNumber(&incomingByte, true); 
            pid[nowMotor].SetTunings(kp[nowMotor],ki[nowMotor],kd[nowMotor]);
            
            break;
            
          case 'X': //rotate angle
            in = myParseNumber(&incomingByte, true);
            target[nowMotor] = ENC[nowMotor].read() + int(angleToCount(in)*CLOCKWISE);
            //Serial.println(target[nowMotor]);
            counting[nowMotor]=true; 
            count[nowMotor]=0; 
            
            break;
    
          case 'G': //motor pwm value btw -255~255
            in = myParseNumber(&incomingByte, true);
            counting[nowMotor] = false;
            output[nowMotor] = in;
            //Serial.println(in);
            
            break;
    
          case 'A': // rotate to angle relative to home
            in = myParseNumber(&incomingByte, true);
            temp2 = ENC[nowMotor].read();
            
            temp = (temp2 - homing[nowMotor])%GRATE;
            if (closestAngle) temp = toClosest(angleToCount(in)-temp, GRATE);
            else temp = angleToCount(in)-temp;
            target[nowMotor] = temp2 + temp;
            //Serial.println("angleToCount:"+String(angleToCount(in))+", enc:"+String(temp2)+", temp:"+String(temp)+", target:"+String(target[nowMotor]));
            counting[nowMotor]=true; 
            count[nowMotor]=0; 
            
            break; 
    
          case 'H': // set now angle as home 0
            //myParseNumber(&incomingByte, true);
            homing[nowMotor] = ENC[nowMotor].read();
            break;

          case 'S': //speed in mm/s
            in = myParseNumber(&incomingByte, true);
            oldPosition = ENC[nowMotor].read();
            oldTime = millis();
            if (in == 0){
              bumping = false;
              velocity = in;
              freqIntegral = 0;
              if (!closestAngle) {
                output[nowMotor] = 0;
                counting[nowMotor] = false;
              }
            } else if (velocity != in){ // check if previous velocity not equal new velocity
              if (velocity == 0) {
                startTime = oldTime;
                freqIntegral = 0;
              }
              velocity = in;
              counting[nowMotor] = true;
            }
            break;

          case 'F':
            freq = myParseNumber(&incomingByte, true);
            if (freq == 0) freqIntegral = 0;
            break;

          case 'Y':
            amplitude = myParseNumber(&incomingByte, true);
            break;
    
          case 'T': 
            prevSpeed = testSpeed;
            testSpeed = myParseNumber(&incomingByte, true);
            
            break;

          case 'R': // encoder speed sample rate
            sampleRate = myParseNumber(&incomingByte, false);
            break;

          case 'W': // waveform
            wave = myParseNumber(&incomingByte, false);
            freqIntegral = 0;
            break;

          case 'O':
            in = myParseNumber(&incomingByte, true);
            if (nowMotor == A) {
              GRATE_S = in*12;
              //Serial.println(String(GRATE_S)+"//");
            } else {
              GRATE = in*12*60./18.;
              //Serial.println(GRATE);
            }
            break;

          case 'C':
            in = myParseNumber(&incomingByte, false);
            if (in == 0) closestAngle = false;
            else closestAngle = true;
            break;

          case 'B':
            readBump(&incomingByte);
            break;

          case 'u': //unity
            connectUnity = true;
            break;

          case 'd': //processing
            connectProcessing = true;
            break;
            
          case 'b': //bye
            lastB = true;
            break;

          case 'e': //print encoder value
            //Serial.println("target:"+String(target[nowMotor])+"enc:"+String(ENC[nowMotor].read()));
            checkEncoder = !checkEncoder;
            break;
            
          case 'L': // pid output limit
            in = myParseNumber(&incomingByte, true);
            pid[nowMotor].SetOutputLimits(-in,in);
            break;

          
      }
   
    }
    Serial.flush();
    if (sendback && connectUnity) Serial.println("k");
    if (cmd == 'b') lastB = true;
  }
  
  
}

void readBump(int *incomingByte) {
  while(*incomingByte == char(-1)) {
    *incomingByte = Serial.read();
  }
  switch(*incomingByte) {
    case 'F': //freq
      *incomingByte = Serial.read();
      //isRead = false;
      bumpFreq = myParseNumber(incomingByte, true);
      //Serial.print(String(bumpFreq));
      break;
    case 'Y': //amp
      *incomingByte = Serial.read();
      //isRead = false;
      bumpAmplitude = myParseNumber(incomingByte, true);
      //Serial.print(String(bumpAmplitude));
      break;
    case 'G': //go
      *incomingByte = Serial.read();
      bumpTime = millis();
      bumping = true;
      break;
  }

  
}

long toClosest(long in, int all) {
  long temp = (in+all)%all;
  if (temp > all/2) temp -= all;
  return temp; 
}

long toClosestAngle(long in, int all) {
  long temp = (in+all)%all;
  if (temp > all/2) temp -= all;
  return temp; 
}

float myParseNumber(int *incomingByte, bool isFloat) {
  float in = 0;
  while (*incomingByte != '\n') {
      //Serial.println("incoming:"+String(incomingByte));
      if (*incomingByte == char(-1)) {
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

  if (isRead) {
    if (isLetter(*incomingByte)) {
      cmd = *incomingByte;
      isRead = false;
    } else cmd = '\0';
    
    char data_char[index+1];

    for (int i=0; i<index; i++) {
      data_char[i] = inData[i];
    }
    
    in = atof(data_char);
    isRead = false;
    index = 0;
  }
  //Serial.print(">>"+String(in)+"\n");
  return in;
}

bool isNumber(char c, bool isFloat) {
  return isFloat ? isDigit(c)||(c=='.')||(c=='-') : isDigit(c)||(c=='-'); 
}

bool isLetter(char c) {
  return (c >= 'a' && c<='z') || (c >= 'A' && c<='Z');
}

bool isUpperLetter(char c) {
  return c >= 'A' && c<='Z';
}

void pwmOut() {
  digitalWrite(STBY, HIGH);
  for (int i=0; i<2; i++) {
    if(output[i] > 0) { 
      digitalWrite(IN1[i],LOW); 
      digitalWrite(IN2[i],HIGH); 
    }
    else {
      digitalWrite(IN1[i],HIGH);
      digitalWrite(IN2[i],LOW);
    }
    analogWrite(PWM[i],abs(output[i]));
  }
}

int angleToCount(long angle) {
  //Serial.println(angle*GRATE);
  //Serial.println((angle*GRATE)/360);
  return (angle*GRATE)/360;
}
