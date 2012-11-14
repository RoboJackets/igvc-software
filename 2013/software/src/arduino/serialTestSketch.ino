

boolean state = false;
long lastMillis;

void setup()
{
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  lastMillis = millis();
}

void loop()
{
  if(Serial.available() > 0)
  {
    int inByte = Serial.read();
    if(inByte == 'T')
    {
      state = !state;
      if(state)
      {
        digitalWrite(13,HIGH);
      }
      else
      {
        digitalWrite(13, LOW);
      }
    }
  } else {
    if((millis() - lastMillis) > 1000)
    {
      Serial.println("test read");
      lastMillis = millis();
    }
  }
}

