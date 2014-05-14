//Clip facing gear box
const int encoderRightData1 = 3;
const int encoderRightData2 = 5;
const int encoderLeftData1 = 2;
const int encoderLeftData2 = 4;
const int rightDir = 9;
const int rightSpeed = 10;
const int rightDisable = 11;
const int leftDir = 7;
const int leftSpeed = 6;
const int leftDisable = 8;

volatile int tickDataRight = 0;
volatile int tickDataLeft = 0;

//CHECK THESE
const float ticksPerRev = 200;
const float wheelCir = 0.092347; // Meters
const float metersPerTick = wheelCir / ticksPerRev;

float desiredSpeedR = 0; // m/s
float desiredSpeedL = 0; // m/s
float actualSpeedR;
float actualSpeedL;

byte PWMOutputR;
byte PWMOutputL;
long ErrorR[10];
long ErrorL[10];
long PIDR;
long PIDL;

float PTerm = 0.3;
float DTerm = 0.05;

// Serial comm vars
#define uchar unsigned char
uchar cmd[4] = {0,0,0,0};
int numBytesIn = 0;
boolean cmdHasStarted = false;
long lastCmdTime;

void setup()
{
  Serial.begin(9600);
  
  pinMode(encoderRightData1, INPUT);
  pinMode(encoderRightData2, INPUT);
  pinMode(encoderLeftData1,  INPUT);
  pinMode(encoderLeftData2,  INPUT);
  
  digitalWrite(encoderRightData1, HIGH);
  digitalWrite(encoderRightData2, HIGH);
  digitalWrite(encoderLeftData1,  HIGH);
  digitalWrite(encoderLeftData2,  HIGH);
  
  pinMode(rightDir, OUTPUT);
  pinMode(rightSpeed, OUTPUT);
  pinMode(rightDisable, OUTPUT);
  pinMode(leftDir, OUTPUT);
  pinMode(leftSpeed, OUTPUT);
  pinMode(leftDisable, OUTPUT);
  attachInterrupt(1, tickRight, CHANGE);
  attachInterrupt(0, tickLeft, CHANGE);
  
  digitalWrite(rightDir, 0); //forward
  digitalWrite(leftDir, 0);
  lastCmdTime = millis();
  
  delay(750);
  
  Serial.println("Ready");
}

void getError()
{
  int i = 0;
  for(i=9;i>=1;i--)
  {
      ErrorR[i] = ErrorR[i-1];
      ErrorL[i] = ErrorL[i-1];
  }
  getSpeed(&actualSpeedL, &actualSpeedR);
  
  ErrorR[0] = desiredSpeedR - actualSpeedR;
  ErrorL[0] = desiredSpeedL - actualSpeedL;
}

void getSpeed(float *left, float *right)
{
  int timeStart = millis();
  int time = millis();
  int tickDataRightS = tickDataRight;
  int tickDataLeftS = tickDataLeft;
  while(time-timeStart<=100)
  {
      time = millis();
  }
  *left = ((metersPerTick*abs(tickDataRight-tickDataRightS))*2)/0.1;
  *right = ((metersPerTick*abs(tickDataLeft-tickDataLeftS))*2)/0.1;
  tickDataRight = 0;
  tickDataLeft = 0;
}

void calculatePID(void)
{

// Calculate the PID  
  PIDR = ErrorR[0]*PTerm;     // start with proportional gain
  PIDL = ErrorL[0]*PTerm; 
  //Accumulator += Error[0];  // accumulator is sum of errors
  //PID += ITerm*Accumulator; // add integral gain and error accumulation
  PIDR += DTerm*(ErrorR[0]-ErrorR[9]);
  PIDL += DTerm*(ErrorL[0]-ErrorL[9]); // differential gain comes next
  //PID = PID>>Divider; // scale PID down with divider

// limit the PID to the resolution we have for the PWM variable

  if(PIDR>=127)
    PIDR = 127;
  if(PIDR<=-126)
    PIDR = -126;
  if(PIDL>=127)
    PIDL = 127;
  if(PIDL<=-126)
    PIDL = -126;
  
  PWMOutputR += PIDR;
  PWMOutputL += PIDL;
  
  if(PWMOutputR>255)
    PWMOutputR = 255;
  if(PWMOutputR<0)
    PWMOutputR = 0;
  if(PWMOutputL>255)
    PWMOutputL = 255;
  if(PWMOutputL<0)
    PWMOutputL = 0;
}

void loop()
{
  while(Serial.available())
  {
    if(Serial.read() == '$')
    {
      desiredSpeedL = Serial.parseFloat();
      desiredSpeedR = Serial.parseFloat();
      lastCmdTime = millis();
      Serial.print('$');
      Serial.print(ErrorL[0]);
      Serial.print(',');
      Serial.println(desiredSpeedL);
    }
  }
  if(lastCmdTime - millis() > 500)
  {
    desiredSpeedL = 0;
    desiredSpeedR = 0;
  }
  getError();
  calculatePID();
  analogWrite(rightSpeed, PWMOutputR);
  analogWrite(leftSpeed, PWMOutputL);
}

void tickRight()
{
  if (digitalRead(encoderRightData1) == digitalRead(encoderRightData2))
  {
    tickDataRight--;
  }
  else
  {
    tickDataRight++;
  }
}

void tickLeft()
{
  if (digitalRead(encoderLeftData1) == digitalRead(encoderLeftData2))
  {
    tickDataLeft--;
  }
  else
  {
    tickDataLeft++;
  }
}
