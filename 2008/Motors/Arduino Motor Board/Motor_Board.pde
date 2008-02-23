
#define ATMEGA168

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/signal.h>
#include <avr/delay.h>
#include <stdio.h>
#include <stdarg.h>


//DIGITAL PINS
  //MOTOR PINS
#define LEFT_MOTOR_PWM_PIN 3
#define LEFT_MOTOR_DIR_PIN 12
#define RIGHT_MOTOR_PWM_PIN 11
#define RIGHT_MOTOR_DIR_PIN 13
  //SWITCH PINS
#define HARD_E_STOP_PIN 7
#define AUTO_MAN_PIN 8  //switch b/w autonomous and manual control
#define PATH_NAV_PIN 9  //switch between the path and nav contests

//ANALOG PINS
  //HALL PINS
#define LOGIC_HALL_PIN 2  //logic current draw
#define RIGHT_HALL_PIN 3  //monitor right motor current draw
#define LEFT_HALL_PIN 4  //monitor left motor current draw
  //VOLTAGE PINS
#define LOGIC_BATTERY_VOLTAGE_PIN 5
#define MOTOR_BATTERY_VOLTAGE_PIN 6

//CONSTANTS
  //MOTOR CONSTANTS
#define FORWARD LOW
#define BACKWARD HIGH
#define ZERO_SPEED 128


//VARIABLES
  //MOTOR VARIABLES
int leftMotorSpeed = ZERO_SPEED;
int rightMotorSpeed = ZERO_SPEED;
  //HALL VARIABLES
int logicHall;
int leftHall;
int rightHall;
  //VOLTAGE VARIABLES
int logicBatteryVoltage;
int motorBatteryVoltage;
  //TIMESTAMP VARIABLES
unsigned long logicTimestamp;
unsigned long leftTimestamp;
unsigned long rightTimestamp;
  //SERIAL VARIABLES
int incomingByte = 0;
  //E-STOP VARIABLES
unsigned char softEStop;
unsigned char hardEStop;
  //SWITCH VARIABLES
unsigned char pathNav;
unsigned char autoMan;

void setup()
{
  //PWM TEST STUFF
//  sbi(TCCR2A,WGM20);
  //TCCR2A |= (1<<WGM20);
  //TCCR2A |= (1<<WGM21);
  TCCR2B |= (1<<CS20);
  TCCR2B &= ~(1<<CS21);
  TCCR2B &= ~(1<<CS22);
/*  TCCR2B |= (1<<CS20);
  TCCR2B &= ~(1<<CS21);
  TCCR2B &= ~(1<<CS22);*/
  
  //TCCR2A &= ~(1<<CS20);
  //TCCR2A &= ~(1<<CS21);
  //TCCR2A &= ~(1<<CS22);
  
  //open the serial port
  Serial.begin(9600);
  
  //set the pin modes
    //motor pins
  pinMode(LEFT_MOTOR_PWM_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_DIR_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR_PIN, OUTPUT);
    //hall pins
  pinMode(HARD_E_STOP_PIN, INPUT);
  pinMode(AUTO_MAN_PIN, INPUT);
  pinMode(PATH_NAV_PIN, INPUT);
  
  //initialize the motors
  leftMotorSpeed = ZERO_SPEED;
  rightMotorSpeed = ZERO_SPEED;
  analogWrite(LEFT_MOTOR_PWM_PIN, leftMotorSpeed);
  digitalWrite(LEFT_MOTOR_DIR_PIN, FORWARD);
  analogWrite(RIGHT_MOTOR_PWM_PIN, rightMotorSpeed);
  digitalWrite(RIGHT_MOTOR_DIR_PIN, FORWARD);
}

void loop()
{
  //analogWrite(LEFT_MOTOR_PWM_PIN, 20);
  //leftMotorSpeed = ZERO_SPEED-1;
  readSwitches();
  readHallSensors();
  readVoltages();
  readSerial();
  setMotors();
}

void readVoltages()
{
  logicBatteryVoltage=analogRead(LOGIC_BATTERY_VOLTAGE_PIN);
  logicBatteryVoltage=analogRead(MOTOR_BATTERY_VOLTAGE_PIN);
}

void readSwitches()
{
  hardEStop=digitalRead(HARD_E_STOP_PIN);
  pathNav=digitalRead(PATH_NAV_PIN);
  autoMan=digitalRead(AUTO_MAN_PIN);
}

void readHallSensors()
{
  logicHall = analogRead(LOGIC_HALL_PIN);
  logicTimestamp = millis();
  rightHall = analogRead(RIGHT_HALL_PIN);
  rightTimestamp = millis();
  leftHall = analogRead(LEFT_HALL_PIN);
  leftTimestamp = millis();
}

void readSerial()
{
  if (Serial.available() > 0)
  {
    incomingByte = Serial.read();
    if (incomingByte == 'w')
    {
      int variableNumber;
      int variableValue;
      while (Serial.available()<2){}
      variableNumber = Serial.read();
      variableValue = Serial.read();
      setVariable(variableNumber, variableValue);
    }
    else if (incomingByte == 'r')
    {
      serialDump();
    }
  }
}
void setMotors()
{
  unsigned char tempSpeed;
  /*if ((softEStop==0)&&(hardEStop==0))
  {*/
  if (softEStop)
  {
    leftMotorSpeed=ZERO_SPEED;
    rightMotorSpeed=ZERO_SPEED;
  }
  if (leftMotorSpeed >= ZERO_SPEED)
  {
    digitalWrite(LEFT_MOTOR_DIR_PIN, FORWARD);
    tempSpeed = leftMotorSpeed - ZERO_SPEED;
    tempSpeed=tempSpeed*2;
    analogWrite(LEFT_MOTOR_PWM_PIN, tempSpeed);
  }
  else
  {
    digitalWrite(LEFT_MOTOR_DIR_PIN, BACKWARD);
    tempSpeed=leftMotorSpeed*2;
    analogWrite(LEFT_MOTOR_PWM_PIN, tempSpeed);
  }
  if (rightMotorSpeed >= ZERO_SPEED)
  {
    digitalWrite(RIGHT_MOTOR_DIR_PIN, FORWARD);
    tempSpeed = rightMotorSpeed - ZERO_SPEED;
    tempSpeed=tempSpeed*2;
    analogWrite(RIGHT_MOTOR_PWM_PIN, tempSpeed);
  }
  else
  {
    digitalWrite(RIGHT_MOTOR_DIR_PIN, BACKWARD);
    tempSpeed=rightMotorSpeed*2;
    analogWrite(RIGHT_MOTOR_PWM_PIN, rightMotorSpeed);
  }
}

void setVariable(int num, int val)
{
  switch (num)
  {
    case 0:
      leftMotorSpeed=val;
    break;
    case 1:
      rightMotorSpeed=val;
    break;
    case 2:
      softEStop=val;
    break;
  }
  Serial.println(num);
}

void serialDump()
{
  /*Serial.print(20, BYTE);
  Serial.print(20, BYTE);
  Serial.print(20, BYTE);
  Serial.print(20, BYTE);
  Serial.print(20, BYTE);
  Serial.print(20, BYTE);
  Serial.print(20, BYTE);
  Serial.print(20, BYTE);
  Serial.print(20, BYTE);
  Serial.print(20, BYTE);
  Serial.print(20, BYTE);*/
  Serial.print(hardEStop, BYTE);
  Serial.print(autoMan, BYTE);
  Serial.print(pathNav, BYTE);
  serialIntegerPrint(leftHall);
  serialIntegerPrint(rightHall);
  serialIntegerPrint(logicBatteryVoltage);
  serialIntegerPrint(motorBatteryVoltage);
}

unsigned char LSByte(int INTEGER)
{
  return INTEGER % 256;
}
unsigned char MSByte(int INTEGER)
{
  return INTEGER >> 8;
}

void serialIntegerPrint(int INTEGER)
{
  unsigned char tempByte;
  tempByte=MSByte(INTEGER);
  Serial.print(tempByte, BYTE);
  tempByte=LSByte(INTEGER);
  Serial.print(tempByte, BYTE);
}
