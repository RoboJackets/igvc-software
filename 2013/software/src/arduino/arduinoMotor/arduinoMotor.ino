

boolean state = false;
long lastMillis;

void setup()
{
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  lastMillis = millis();
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
    
    if(inByte == "W")
    {
      int pwm = Serial.read();
      analogWrite(9, pwm);
    }
    
    if(inByte == "S")
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

