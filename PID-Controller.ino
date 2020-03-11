
#include <Encoder.h>
// Encoders
Encoder EncHorizontal(12,13);
Encoder EncVertical(3,4);

long newPositionHorizontal;
long oldPositionHorizontal  = -999;
long newPositionVertical;
long oldPositionVertical  = -999;
/*******Pin Definitions on Arduino UNO*******/

// Motors (connecting to TB6612FNG)
//Motor 1
const int pinAIN1 = 8; //Direction
const int pinAIN2 = 7; //Direction
const int pinPWMA = 6; //Voltage

//Motor 2
const int pinBIN1 = 10; //Direction
const int pinBIN2 = 9; //Direction
const int pinPWMB = 5; //Voltage

//Standby
const int pinSTBY = 11;

//Constants
const int gearRatio = 298;

//Encoder Variables
const unsigned short acceptableCountError = 0;
const unsigned short activationCountError = 1;
bool motorActive;
bool motor2Active;
bool encoder1OutA_CurState, encoder1OutB_CurState, encoder1OutA_PrevState, encoder1OutB_PrevState;
bool encoder2OutA_CurState, encoder2OutB_CurState, encoder2OutA_PrevState, encoder2OutB_PrevState;

//PID control
int targetCount = 0;
int targetCount2 = 0;
int curCount = 0;
int curCount2 = 0;
long integral;
long integral2;

//tuning guidelines: (Voltage feed = Kp*(error) + Ki*error.usPassed + Kd*encoderSpeed)
const int KiActivationErrorRange = 100;
const int KiRange2 = 10;
const float KiRange2Factor = 2;
const int KdActivationErrorRange = 100;

//Kp: error approx 100(small step / 10 motor rounds) -> 10,000(big step) -> around 2.5E-1
//Ki: error approx 1.0E3, total us in transition approx 1.0E6 -> 2.5E-7
//Kd: approx 1.0E-1 encoder counts/us -> 2.5E+3
//const float Kp = 2.5e-1;
//const float Ki = 5e-7;
//const float Kd = -5e3; // negative here to compensate for different speeds when approaching target

const float Kp = 2;
const float Ki = 0.00001;
const float Kd = 0; // negative here to compensate for different speeds when approaching target

//global variables
unsigned long usPassed;
unsigned long usPassed2;
float encoderSpeed = 0;
float encoderSpeed2 = 0;

//Motor variables
int voltageFeed;
int voltageFeed2;
const bool turnDirection = true;  //for motorDrive function
const bool turnDirection2 = true;
const bool motor1 = 0;  //for motorDrive, motorStop, motorBrake functions
const bool motor2 = 1;  //for motorDrive, motorStop, motorBrake functions

String incomingStr;
String vDirection;
String hDirection;
String Direction;
float encoderValue = 89.4f;
int maximalVoltage = 255;

void setup()
{
  Serial.begin(115200); //Sets the data rate in bits per second (baud) for serial data transmission. For communicating with the computer, use one of these rates: 300, 600, 1200, 2400, 4800, 9600, 19200, 28800, 38400, 57600, or 115200
  randomSeed(0);
  //Set the PIN Modes
  pinMode(pinPWMA, OUTPUT);
  pinMode(pinAIN1, OUTPUT);
  pinMode(pinAIN2, OUTPUT);

  pinMode(pinPWMB, OUTPUT);
  pinMode(pinBIN1, OUTPUT);
  pinMode(pinBIN2, OUTPUT);

  pinMode(pinSTBY, OUTPUT);

}

void loop()
{
  readEncoder();
  getusPassed();
  getEncoderSpeed();
  getVoltageFeedFromPID();
  updateMotor();
}

void serialEvent() { //called when data from serial port is available
  char tenDigits;  //十位數
  char digit;      //個位數
  char decPlace;  //小數點後第一位
  int stringLength;
  //0 => 48, 1 => 49, ... , 9 => 57, - => 45
  incomingStr = Serial.readStringUntil('\n');
  stringLength = incomingStr.length();
  Direction = incomingStr[0];

    //個位數有小數點
    if (stringLength == 2) {
      digit = incomingStr[1];
      moveToBalanceLevel(digit - '0');
    }
    //十位數無小數點 or 45度個位數無小數點
    if (stringLength == 3) {
      if ((incomingStr[0] == 'u' || incomingStr[0] == 'd') && (incomingStr[1] == 'r' || incomingStr[1] == 'l' )) {
        Direction = incomingStr[0];
        Direction += incomingStr[1];
        digit = incomingStr[2];
        moveToBalanceLevel(digit - '0');
      }
      else {
        tenDigits = incomingStr[1];
        digit = incomingStr[2];
        moveToBalanceLevel((tenDigits - '0') * 10 + (digit - '0'));
      }
    }
    //個位數值有小數點 or 45度十位數無小數點
    if (stringLength == 4) {
      if ((incomingStr[0] == 'u' || incomingStr[0] == 'd') && (incomingStr[1] == 'r' || incomingStr[1] == 'l' )) {
        Direction = incomingStr[0];
        Direction += incomingStr[1];
        tenDigits = incomingStr[2];
        digit = incomingStr[3];
        moveToBalanceLevel((tenDigits - '0') * 10 + (digit - '0'));
      }
      else {
        digit = incomingStr[1];
        decPlace = incomingStr[3];
        moveToBalanceLevel(float(digit - '0') + (float(decPlace - '0') / 10));
      }
    }
    //十位數值有小數點 or 45度個位數有小數點
    if (stringLength == 5) {
      if ((incomingStr[0] == 'u' || incomingStr[0] == 'd') && (incomingStr[1] == 'r' || incomingStr[1] == 'l' )) {
        Direction = incomingStr[0];
        Direction += incomingStr[1];
        digit = incomingStr[2];
        decPlace = incomingStr[4];
        moveToBalanceLevel(float(digit - '0') + (float(decPlace - '0') / 10));
      }
      else {
        tenDigits = incomingStr[1];
        digit = incomingStr[2];
        decPlace = incomingStr[4];
        moveToBalanceLevel(float((tenDigits - '0') * 10) + (float(digit - '0')) + (float(decPlace - '0') / 10));
      }
    }
    // 45度十位數有小數點
    if (stringLength == 6) {
      Direction = incomingStr[0];
      Direction += incomingStr[1];
      tenDigits = incomingStr[2];
      digit = incomingStr[3];
      decPlace = incomingStr[5];
      moveToBalanceLevel(float((tenDigits - '0') * 10) + (float(digit - '0')) + (float(decPlace - '0') / 10));
    }
  

}

void moveToBalanceLevel(float bal)
{
  Serial.println(bal);
  // move right or left
  if (Direction == "r" || Direction == "l") {
    if (Direction == "r") {
      targetCount = bal * encoderValue;
    }
    else if (Direction == "l") {
      targetCount = -bal * encoderValue;
    }
    integral = 0; //reset integral in PID control
    motorActive = true;
  }

  // move up or down
  if (Direction == "u" || Direction == "d") {
    if (Direction == "u") {
      targetCount2 = bal * encoderValue;
    }
    else if (Direction == "d") {
      targetCount2 = -bal * encoderValue;
    }
    integral2 = 0; //reset integral in PID control
    motor2Active = true;
  }

  // move 45 degree
  if (Direction == "ur" || Direction == "ul" || Direction == "dr" || Direction == "dl") {
    if (Direction == "ur") {
      targetCount =  bal * encoderValue;
      targetCount2 = bal * encoderValue;
    }
    if (Direction == "ul") {
      targetCount = -bal * encoderValue;
      targetCount2 = bal * encoderValue;
    }
    if (Direction == "dr") {
      targetCount =  bal * encoderValue;
      targetCount2 = -bal * encoderValue;
    }
    if (Direction == "dl") {
      targetCount =  -bal * encoderValue;
      targetCount2 = -bal * encoderValue;
    }
    integral = 0; //reset integral in PID control
    motorActive = true;
    integral2 = 0; //reset integral in PID control
    motor2Active = true;
  }
}

void getVoltageFeedFromPID()
{
  int error;
  float I, D;

  error = targetCount - newPositionHorizontal;
  // Integral
  if (abs(error) < KiActivationErrorRange) {
    integral += error * usPassed;
    if (abs(error) < KiRange2) {
      I = Ki * KiRange2Factor * integral;
    } else {
      I = Ki * integral;
    }
  } else {
    integral = 0;
    I = 0;
  }
  //Derivative
  if (abs(error) < KdActivationErrorRange) {
    D = Kd * encoderSpeed;
  } else {
    D = 0;
  }

  //Derive driving voltage
  if (abs(targetCount - newPositionHorizontal) > activationCountError) {
    motorActive = true;
  }
  if (abs(targetCount - newPositionHorizontal) > acceptableCountError && motorActive) { // after motor has reached acceptableCountError, the error needs to excceed activationCountError to start the motor again.
    voltageFeed = Kp * (error) + I + D;

    if (voltageFeed > 0) {
      if (voltageFeed > maximalVoltage) {
        voltageFeed = maximalVoltage;
      }
      //motorDrive(motor1, turnDirection, voltageFeed);
    } else {
      if (voltageFeed < -maximalVoltage) {
        voltageFeed = -maximalVoltage;
      }
      //motorDrive(motor1, !turnDirection, -voltageFeed);
    }
  } else {
    integral = 0;
    voltageFeed = 0;
  }

  /////////////////
  int error2;
  float I2, D2;

  error2 = targetCount2 - newPositionVertical;
  // Integral
  if (abs(error2) < KiActivationErrorRange) {
    integral2 += error2 * usPassed2;
    if (abs(error2) < KiRange2) {
      I2 = Ki * KiRange2Factor * integral2;
    } else {
      I2 = Ki * integral2;
    }
  } else {
    integral2 = 0;
    I2 = 0;
  }
  //Derivative
  if (abs(error2) < KdActivationErrorRange) {
    D2 = Kd * encoderSpeed2;
  } else {
    D2 = 0;
  }

  //Derive driving voltage
  if (abs(targetCount2 - newPositionVertical) > activationCountError) {
    motor2Active = true;
  }
  if (abs(targetCount2 - newPositionVertical) > acceptableCountError && motor2Active) { // after motor has reached acceptableCountError, the error needs to excceed activationCountError to start the motor again.
    voltageFeed2 = Kp * (error2) + I2 + D2;

    if (voltageFeed2 > 0) {
      if (voltageFeed2 > maximalVoltage) {
        voltageFeed2 = maximalVoltage;
      }
      //motorDrive(motor1, turnDirection, voltageFeed);
    } else {
      if (voltageFeed2 < -maximalVoltage) {
        voltageFeed2 = -maximalVoltage;
      }
      //motorDrive(motor1, !turnDirection, -voltageFeed);
    }
  } else {
    integral2 = 0;
    voltageFeed2 = 0;
  }
}

void updateMotor() {
  if (Direction == "r" || Direction == "l") {
    if (voltageFeed > 0) {
      motorDrive(motor1, turnDirection, voltageFeed);
    } else if (voltageFeed < 0) {
      motorDrive(motor1, !turnDirection, -voltageFeed);
    }
    else  {
      motorActive = false;
      motorStop(motor1);
      motorsStandby();
    }
  }
  else if (Direction == "u" || Direction == "d") {
    if (voltageFeed2 > 0) {
      motorDrive(motor2, turnDirection2, voltageFeed2);
    } else if (voltageFeed2 < 0) {
      motorDrive(motor2, !turnDirection2, -voltageFeed2);
    }
    else {
      motor2Active = false;
      motorStop(motor2);
      motorsStandby();
    }
  }

  else if (Direction == "ur" || Direction == "dr" || Direction == "ul" || Direction == "dl") {
    if (voltageFeed > 0) {
      motorDrive(motor1, turnDirection, voltageFeed);
    } else if (voltageFeed < 0) {
      motorDrive(motor1, !turnDirection, -voltageFeed);
    }
    else  {
      motorActive = false;
      motorStop(motor1);
      motorsStandby();
    }
    if (voltageFeed2 > 0) {
      motorDrive(motor2, turnDirection2, voltageFeed2);
    } else if (voltageFeed2 < 0) {
      motorDrive(motor2, !turnDirection2, -voltageFeed2);
    }
    else {
      motor2Active = false;
      motorStop(motor2);
      motorsStandby();
    }
  }
}

void getusPassed() {
  static unsigned long prevTime = 0;
  unsigned long curTime;
  static unsigned long prevTime2 = 0;
  unsigned long curTime2;

  curTime = micros();
  usPassed = curTime - prevTime;
  prevTime = curTime;

  curTime2 = micros();
  usPassed2 = curTime2 - prevTime2;
  prevTime2 = curTime2;
}

void getEncoderSpeed() {
  static unsigned long usPassedBetweenEncoderReadings = 0;
  static int prevCount = 0;
  float newSpeed;
  static unsigned long usPassed2BetweenEncoderReadings = 0;
  static int prevCount2 = 0;
  float newSpeed2;

  usPassedBetweenEncoderReadings += usPassed;
  if (usPassed > 1000) {
    newSpeed = (float)(curCount - prevCount) * 0.001;
    prevCount = curCount;
    usPassedBetweenEncoderReadings -= 1000;
    encoderSpeed = encoderSpeed * 0.8 + (newSpeed) * 0.2;
  }

  usPassed2BetweenEncoderReadings += usPassed2;
  if (usPassed2 > 1000) {
    newSpeed2 = (float)(curCount2 - prevCount2) * 0.001;
    prevCount2 = curCount2;
    usPassed2BetweenEncoderReadings -= 1000;
    encoderSpeed2 = encoderSpeed2 * 0.8 + (newSpeed2) * 0.2;
  }
}

void readEncoder()
{

   newPositionHorizontal = EncHorizontal.read();
  if (newPositionHorizontal != oldPositionHorizontal) {
    oldPositionHorizontal = newPositionHorizontal;
    Serial.println(newPositionHorizontal);
  }
  
   newPositionVertical = EncVertical.read();
  if (newPositionVertical != oldPositionVertical) {
    oldPositionVertical = newPositionVertical;
    Serial.println(newPositionVertical);
  }
}










void motorDrive(bool motorNumber, bool motorDirection, unsigned short motorVoltage)
{
  /*
    This Drives a specified motor, in a specific direction, at a specified Voltage:
    - motorNumber: motor1 or motor2 ---> Motor 1 or Motor 2
    - motorDirection: turnCW or turnCCW ---> clockwise or counter-clockwise
    - motorVoltage: 0 to 255 ---> 0 = stop / 255 = fast
  */

  bool pinIn1;  //Relates to AIN1 or BIN1 (depending on the motor number specified)


  //Specify the Direction to turn the motor
  //Clockwise: AIN1/BIN1 = HIGH and AIN2/BIN2 = LOW
  //Counter-Clockwise: AIN1/BIN1 = LOW and AIN2/BIN2 = HIGH
  if (motorDirection == true)
    pinIn1 = HIGH;
  else
    pinIn1 = LOW;

  //Select the motor to turn, and set the direction and the Voltage
  if (motorNumber == motor1)
  {
    digitalWrite(pinAIN1, pinIn1);
    digitalWrite(pinAIN2, !pinIn1);  //This is the opposite of the AIN1
    analogWrite(pinPWMA, motorVoltage);
  }
  else
  {
    digitalWrite(pinBIN1, !pinIn1);
    digitalWrite(pinBIN2, pinIn1);  //This is the opposite of the BIN1
    analogWrite(pinPWMB, motorVoltage);
  }



  //Finally , make sure STBY is disabled - pull it HIGH
  digitalWrite(pinSTBY, HIGH);

}

void motorBrake(bool motorNumber)
{
  /*
    This "Short Brake"s the specified motor, by setting Voltage to zero
  */

  if (motorNumber == motor1)
    analogWrite(pinPWMA, 0);
  else
    analogWrite(pinPWMB, 0);

}


void motorStop(bool motorNumber)
{
  /*
    This stops the specified motor by setting both IN pins to LOW
  */
  if (motorNumber == motor1) {
    digitalWrite(pinAIN1, LOW);
    digitalWrite(pinAIN2, LOW);
  }
  else
  {
    digitalWrite(pinBIN1, LOW);
    digitalWrite(pinBIN2, LOW);
  }
}


void motorsStandby()
{
  /*
    This puts the motors into Standby Mode
  */
  digitalWrite(pinSTBY, LOW);
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
