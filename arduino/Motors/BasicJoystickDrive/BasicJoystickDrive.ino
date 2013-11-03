//Pindefs
int rightDir = 9;
int rightSpeed = 10;
int rightDisable = 11;
int leftDir = 7;
int leftSpeed = 6;
int leftDisable = 8;


void setup()
{
  Serial.begin(9600);
  
  pinMode(rightDir, OUTPUT);
  pinMode(rightSpeed, OUTPUT);
  pinMode(rightDisable, OUTPUT);
  
  pinMode(leftDir, OUTPUT);
  pinMode(leftSpeed, OUTPUT);
  pinMode(leftDisable, OUTPUT);
  
  digitalWrite(leftDisable, LOW);
  digitalWrite(rightDisable, LOW);
  
  Serial.flush();
  
  Serial.println("Ready");
}

void loop()
{
  if(Serial.available() > 0)
  {
    if(Serial.read() == 'S')
    {
      while(Serial.available()<=0){}
      if(Serial.read() == 'W')
      {
        while(Serial.available()<=0){}
        digitalWrite(rightDir, Serial.parseInt());
        while(Serial.available()<=0){}
        analogWrite(rightSpeed, Serial.parseInt());
        while(Serial.available()<=0){}
        digitalWrite(leftDir, Serial.parseInt());
        while(Serial.available()<=0){}
        analogWrite(leftSpeed, Serial.parseInt());
      }
    }
  }
}
