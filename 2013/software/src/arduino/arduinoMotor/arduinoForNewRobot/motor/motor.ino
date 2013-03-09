

#include <stdlib.h>
#include "PID_v1.h"

//pin numbers
const int rightDir = 5;
const int rightSpeed = 6;
const int rightDisable = 7;
const int leftDir = 2;
const int leftSpeed = 3;
const int leftDisable = 4;
const int encoderRightData1;
const int encoderRightData2;
const int encoderLeftData1;
const int encoderLeftData2;
const int MaxMotorSpeed;
double DesiredSpeedLeft, PIDSpeedInLeft, PIDSpeedOutLeft; //PID Variables Left Motor
double DesiredSpeedRight, PIDSpeedInRight, PIDSpeedOutRight; //PID Variables Right Motor
const double R_Kp=2, R_Ki=5, R_Kd=1;
const double L_Kp=2, L_Ki=5, L_Kd=1;

int dirRight;
int dirLeft;
int byte0;
int byte1;
int byte2;
int byte3;
int byte4;
int byte5;
int byte6;
double byte7;
double byte8;

PID PIDLeft(&PIDSpeedInLeft, &PIDSpeedOutLeft, &DesiredSpeedLeft, L_Kp,L_Ki,L_Kd, DIRECT); //Calling PID Method Left Motor
PID PIDRight(&PIDSpeedInRight, &PIDSpeedOutRight, &DesiredSpeedRight, R_Kp,R_Ki,R_Kd, DIRECT); //Calling PID Method Right Motor

volatile int tickDataRight = 0;
volatile int tickDataLeft = 0;

const float ticksPerRev = 200;
const float wheelCir = 0.79756; //Meters
const float metersPerTick = wheelCir / ticksPerRev;
const float wheelBase = 0.762;

void setup()
{
  Serial.begin(9600);
  pinMode(rightDir, OUTPUT);
  pinMode(rightSpeed, OUTPUT);
  pinMode(rightDisable, OUTPUT);
  pinMode(leftDir, OUTPUT);
  pinMode(leftSpeed, OUTPUT);
  pinMode(leftDisable, OUTPUT);
  pinMode(encoderRightData1, INPUT);
  pinMode(encoderRightData2, INPUT);
  pinMode(encoderLeftData1, INPUT);
  pinMode(encoderLeftData2, INPUT);
  attachInterrupt(0, tickRight, RISING);
  attachInterrupt(1, tickLeft, RISING);
  PIDLeft.SetMode(AUTOMATIC);
  PIDRight.SetMode(AUTOMATIC);
}

void loop()
{
  if (Serial.available()>0)
  {
    byte0 = Serial.read();
    if (byte0 == 'S')
    {
      while (Serial.available()<=0){}
      byte1 = Serial.read(); //First Case either W, R, D, T
      delay(200);
      if (byte1 == 'W' || byte1 == 'D' || byte1 == 'P')
      {
        while (Serial.available()<=0){}
        byte3 = Serial.parseInt(); //Right Direction
        while (Serial.available()<=0){}
        byte4 = Serial.parseInt(); //Right Speed
        DesiredSpeedRight=(double)byte4;
        while (Serial.available()<=0){}
        byte5 = Serial.parseInt(); //Left Direction
        while (Serial.available()<=0){}
        byte6 = Serial.parseInt(); //Left Speed
        DesiredSpeedLeft=(double)byte6;
        if (byte1 == 'D')
        {
          while (Serial.available()<=0){}
          byte7 = Serial.parseFloat();
        }
      if (byte1 == 'R')
      {
        while (Serial.available()<=0){}
        byte8 == Serial.parseFloat();
      }
      else 
      {
        return;
      }
    } 
    else 
    {
      return;
    }
  }
  else
  {
    return;
  }
  
  switch (byte1)
  {
    case 'W': 
    {
      //moving wheels in a certain direction and speed
      digitalWrite(rightDir, byte3);
      digitalWrite(leftDir, byte5);
      analogWrite(rightSpeed, byte4);
      analogWrite(leftSpeed, byte6);
      break;
    }
    case 'D': //
    {
      digitalWrite(rightDir, byte3);
      digitalWrite(leftDir, byte5);
      analogWrite(rightSpeed, byte4);
      analogWrite(leftSpeed, byte6);
      
      //Loop for moving a set distance
      tickDataRight = 0;
      tickDataLeft = 0;
      double distTravelledRight = (metersPerTick*abs(tickDataRight))*2;
      double distTravelledLeft = (metersPerTick*abs(tickDataLeft))*2;
      double distTravelled = (distTravelledRight+distTravelledLeft)/2.0;
      while(distTravelled<=byte7)
      {
        distTravelledRight = (metersPerTick*abs(tickDataRight))*2;
        distTravelledLeft = (metersPerTick*abs(tickDataLeft))*2;
        distTravelled = (distTravelledRight+distTravelledLeft)/2.0;
      }
      
      analogWrite(rightSpeed, 0);
      analogWrite(leftSpeed, 0);
      Serial.println("!");
      break; 
    }
    
    case 'R':
    {
      //Turning an angle
      tickDataRight = 0;
      tickDataLeft = 0;
      double distTravelledRight = (metersPerTick*abs(tickDataRight))*2;
      double distTravelledLeft = (metersPerTick*abs(tickDataLeft))*2;
      double angle = abs(distTravelledRight-distTravelledLeft)/wheelBase;
      
      while(angle<=byte8) 
      {
        distTravelledRight = (metersPerTick*abs(tickDataRight))*2;
        distTravelledLeft = (metersPerTick*abs(tickDataLeft))*2;
        angle = abs(distTravelledRight-distTravelledLeft)/wheelBase;
      };
      
      analogWrite(rightSpeed, 0);
      analogWrite(leftSpeed, 0);

      break;
    }
    
    case 'T':
    {
      //Testing the connection to arduinos will send a confirmaton to the computer
      Serial.println("T");
      break;
    }
    
    case 'P':
    {
      tickDataRight = 0;
      tickDataLeft = 0;
      int timeStart = millis();
      int time = millis();
      digitalWrite(rightDir, byte3);
      digitalWrite(leftDir, byte5);
      
      while(time-timeStart<=100)
      {
        time = millis();
      }
      
      double distTravelledRight = (metersPerTick*abs(tickDataRight))*2;
      double distTravelledLeft = (metersPerTick*abs(tickDataLeft))*2;
      double speedRight = distTravelledRight/0.1;
      double speedLeft = distTravelledLeft/0.1;
      
      PIDSpeedInRight=speedRight/MaxMotorSpeed*255; //Scale Motor Encoder Speed Reading to 0-255 to compare to PWM Desired Command
      PIDSpeedInLeft=speedLeft/MaxMotorSpeed*255;
      PIDLeft.SetControllerDirection(byte5); //Update Direction in PID Controller
      PIDLeft.Compute();                     //PID Controller will compute new variables, called earlier by reference.
      PIDRight.SetControllerDirection(byte3);
      PIDRight.Compute();
      analogWrite(rightSpeed, PIDSpeedOutRight);
      analogWrite(leftSpeed, PIDSpeedOutLeft);
      
      break;  
    }
    
  }
}
}

void tickRight()
{
  if (digitalRead(encoderRightData1) == digitalRead(encoderRightData2))
  {
    tickDataRight++;
  }
  else
  {
    tickDataRight--;
  }
}

void tickLeft()
{
  if (digitalRead(encoderLeftData1) == digitalRead(encoderLeftData2))
  {
    tickDataLeft++;
  }
  else
  {
    tickDataLeft--;
  }
}
      
