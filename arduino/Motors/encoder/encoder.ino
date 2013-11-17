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
const float wheelCir = 0.092347; //Meters
const float metersPerTick = wheelCir / ticksPerRev;

//int desiredSpeed = 50; //0-255 PWM
float desiredSpeed = 0.5; //m/s
//int desiredSpeed = map(desiredSpeed,0,topSpeed,0,255); m/s
float actualSpeedR;
float actualSpeedL;
//int topSpeed = 10; //m/s

byte PWMOutputR;
byte PWMOutputL;
long ErrorR[10];
long ErrorL[10];
long PIDR;
long PIDL;

int PTerm = 0.3;
float DTerm = 0.05;

void setup()
{
  Serial.begin(9600);
  
  pinMode(encoderRightData1, INPUT);
  pinMode(encoderRightData2, INPUT);
  pinMode(encoderLeftData1, INPUT);
  pinMode(encoderLeftData2, INPUT);
  
  digitalWrite(encoderRightData1, 1);
  digitalWrite(encoderRightData2, 1);
  digitalWrite(encoderLeftData1, 1);
  digitalWrite(encoderLeftData2, 1);
  
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
}

void getError()
{
  byte i = 0;
  for(i=0;i<9;i++)
  {
      ErrorR[i+1] = ErrorR[i];
      ErrorL[i+1] = ErrorL[i];
  }
  float speeds[2];
  getSpeed(speeds);
  actualSpeedR = speeds[0];
  actualSpeedL = speeds[1];
  
  //actualSpeedR = map(speeds[0],0,topSpeed,0,255);
  //actualSpeedL = map(speeds[1],0,topSpeed,0,255);
  ErrorR[0] = desiredSpeed - actualSpeedR;
  ErrorL[0] = desiredSpeed - actualSpeedL;
}

void getSpeed(float speeds1[])
{
  int timeStart = millis();
  int time = millis();
  int tickDataRightS = tickDataRight;
  int tickDataLeftS = tickDataLeft;
  while(time-timeStart<=100)
  {
      time = millis();
  }
  float retR = ((metersPerTick*abs(tickDataRight-tickDataRightS))*2)/0.1;
  float retL = ((metersPerTick*abs(tickDataLeft-tickDataLeftS))*2)/0.1;
  tickDataRight = 0;
  tickDataLeft = 0;
  speeds1[0] = retR;
  speeds1[1] = retL;
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
  
  getError();
  calculatePID();
  analogWrite(rightSpeed, PWMOutputR);
  analogWrite(leftSpeed, PWMOutputL);
  //Serial.print("R:");
  //Serial.print(actualSpeedR);
  Serial.print("PWM:");
  Serial.print(PWMOutputL);
  Serial.print("  L:");
  Serial.print(actualSpeedL);
  Serial.println();
  
  //Serial.println(tickDataLeft);
  
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
