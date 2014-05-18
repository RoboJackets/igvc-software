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

const float ticksPerRev = 1600.0;
const float wheelCir = 1.11715; // Meters
const float metersPerTick = wheelCir / ticksPerRev;
const float DEADBAND = 10;

float desiredSpeedR = 0; // m/s
float desiredSpeedL = 0; // m/s
float actualSpeedR;
float actualSpeedL;

float lastErrorL;
float lastErrorR;

float P = 13;
float D = 0;

int PWM_L = 0;
int PWM_R = 0;

long lastLoopTime;

// Serial comm vars
#define uchar unsigned char
uchar cmd[4] = {
  0,0,0,0};
int numBytesIn = 0;
boolean cmdHasStarted = false;
long lastCmdTime;

/* TODO
 * 1 - fix deadband (maybe bigger?)
 * 2 - push command speed
 * 3 - fix timeout
 */


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

  delay(1000);

  lastCmdTime = millis();
  lastLoopTime = millis();

  digitalWrite(rightDir, 0);
  analogWrite(rightSpeed, 255);

  Serial.println();
  Serial.flush();
  Serial.println("Ready");
}

boolean gotCommand = false;

void loop()
{
  gotCommand = false;
  long s = millis();
  while(Serial.available())
  {
    if(Serial.read() == '$')
    {
      desiredSpeedL = Serial.parseFloat();
      desiredSpeedR = Serial.parseFloat();
      lastCmdTime = millis();
      gotCommand = true;
    }
  }
  long e = millis();
  if(gotCommand)
  {
    Serial.print('$');
    Serial.print(actualSpeedL);
    Serial.print(',');
    Serial.print(actualSpeedR);
    Serial.print('\n');
  }
  //Serial.println(tickDataLeft);
  if( millis() - lastCmdTime > 500)
  {
    Serial.println("TIMEOUT");
    desiredSpeedL = 0;
    desiredSpeedR = 0;
  }

  float dT_sec = (float)( millis() - lastLoopTime ) / 1000.0;
  lastLoopTime = millis();
  actualSpeedL = ( metersPerTick * tickDataLeft ) / dT_sec;
  actualSpeedR = ( metersPerTick * tickDataRight ) / dT_sec;

  tickDataLeft = 0;
  tickDataRight = 0;

  delay(50);

  float ErrorL = desiredSpeedL - actualSpeedL;
  float ErrorR = desiredSpeedR - actualSpeedR;

  float dErrorL = ErrorL - lastErrorL;
  float dErrorR = ErrorR - lastErrorR;

  int dPWM_L = (int)( P * ErrorL + D * dErrorL );
  int dPWM_R = (int)( P * ErrorR + D * dErrorR );

  PWM_L += dPWM_L;
  PWM_R += dPWM_R;

  PWM_L = min(255, max(-255, PWM_L) );
  PWM_R = min(255, max(-255, PWM_R) );

  // Deadband
  if( abs(PWM_L) < DEADBAND )
    PWM_L = 0;
  if( abs(PWM_R) < DEADBAND )
    PWM_R = 0;

  int dirL = PWM_L < 0;
  int dirR = PWM_R < 0;

  int powerL = dirL ? 255 + PWM_L : PWM_L;
  int powerR = dirR ? 255 + PWM_R : PWM_R;

  digitalWrite(rightDir, dirR);
  digitalWrite(leftDir, dirL);
  analogWrite(rightSpeed, powerR);
  analogWrite(leftSpeed, powerL);
  
  Serial.print(tickDataLeft);
  Serial.print('\t');
  Serial.println(tickDataRight);

  lastErrorL = ErrorL;
  lastErrorR = ErrorR;
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

