#include <stdlib.h>

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
      if (byte1 == 'W' || byte1 == 'D')
      {
        while (Serial.available()<=0){}
        byte3 = Serial.parseInt(); //Right Direction
        while (Serial.available()<=0){}
        byte4 = Serial.parseInt(); //Right Speed
        while (Serial.available()<=0){}
        byte5 = Serial.parseInt(); //Left Direction
        while (Serial.available()<=0){}
        byte6 = Serial.parseInt(); //Left Speed
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
    case 'D':
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
      distTravelled = (distTravelledRight+distTravelledLeft)/2.0;
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
    
    case 'P'
    {
      
    
      break;  
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
      
