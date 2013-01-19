//STUFF ARE IN METEORS aka meters
//WORKS ON OLD ROBOT
long lastMillis;
double dist;
volatile int tickData = 0;
volatile int tickData2 = 0;
const float ticksPerRev = 24.0;
const float wheelCir = 0.79756; //Meters
const float metersPerTick = wheelCir / ticksPerRev;
const float baseLine = 0.762; //Meters
const float initalAngle = 0;
boolean dir1;
boolean dir2;
float distTravelled;

void setup()
{
  Serial.begin(9600);
  pinMode(13, OUTPUT);  //onboard LED
  pinMode(9, OUTPUT);   //motor pwm pin
  pinMode(10, OUTPUT);  //motor direction pin
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4,INPUT);
  pinMode(5,INPUT);
  attachInterrupt(1,tick,CHANGE);
  attachInterrupt(0,tick,CHANGE);
}

void loop()
{
  if(Serial.available() > 0)
  {
    int inByte = Serial.read();
    if(inByte == 'T')
    {
      Serial.println("T");
    }
    
    if(inByte == 'W')
    {
      int inByte = Serial.read();
      if(inByte == 'D')
      {
        dist = Serial.parseFloat();
        int dir = Serial.read();
        int pwm = Serial.read();
        digitalWrite(10, dir);
        analogWrite(9, pwm);
        while (distTravelled < dist)
        {
          float distTravelled1 = (metersPerTick*abs(tickData))*0.004385965; //random number to get to correct distance as I do not know how many ticksPerRev are on the old encoders
          float distTravelled2 = (metersPerTick*abs(tickData2))*0.004385965;
          distTravelled = (distTravelled1+distTravelled2)/2;
        }
        Serial.println("!");
        analogWrite(9, 0);
        
      }    
      if(inByte == 'S')
      {  
        int dir = Serial.read();
        digitalWrite(10, dir);
        int pwm = Serial.read();
        analogWrite(9, pwm);
      }
    }
    
    if(inByte == 'S')
    {
      analogWrite(9, 0);
    }
    
        
  } 
  
  else 
  {
    if((millis() - lastMillis) > 1000)
    {
      Serial.println("Running");
      lastMillis = millis();
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

