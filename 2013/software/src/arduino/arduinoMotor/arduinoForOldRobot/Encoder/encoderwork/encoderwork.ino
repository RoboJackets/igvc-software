long lastMillis;
//double dist;
volatile int tickData = 0;
volatile int tickData2 = 0;
const float ticksPerRev = 24.0;
const float wheelCir = 0.79756; //Meters
const float metersPerTick = wheelCir / ticksPerRev;
const float baseLine = 0.762; //Meters
const float initalAngle = 0;
boolean dir1;
boolean dir2;
double distTravelled;

double dist;
int byte1;
int byte2;
int byte3;
int byte4;
int byte5;
int byte6;

void setup()
{
  Serial.begin(9600);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  attachInterrupt(0, tick, RISING);
  attachInterrupt(1, tick, RISING);
}

void loop()
{
 // while (Serial.available()<=0){}
  while (Serial.available()>0)
  {
    byte1 = Serial.read(); //First Case either W, R, S, T
  //  delay(200);
 //   dist = Serial.parseFloat();
  }
  //delay(10);
  switch (byte1)
  {
    case 'R':
    {
      double distTravelled1 = (metersPerTick*abs(tickData))*0.004385965*2;
      double distTravelled2 = (metersPerTick*abs(tickData2))*0.004385965*2;
      distTravelled = (distTravelled1+distTravelled2)/2.0;
      Serial.println(distTravelled1);
  //    delay(10);
      break;
    }
    case 'F':
    {
      tickData = 0;
      tickData2 = 0;
      //Serial.println(tickData);
      break;
    }
    case 'T':
    {
      //Testing the connection to arduinos will just return a confirmaton to the computer
      Serial.println("T");
      break;
    }
  }
  
}

void tick()
{
  if (digitalRead(3) == digitalRead(5))
  {
    dir1 = true;
    tickData++;
  }
  else
  {
    dir1 = false;
    tickData--;
  }
  if (digitalRead(2) == digitalRead(4))
  {
    dir2 = true;
    tickData2++;
  }
  else
  {
    dir2 = false;
    tickData2--;
  }
}
      
