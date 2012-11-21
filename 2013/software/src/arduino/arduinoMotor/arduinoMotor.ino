long lastMillis;

void setup()
{
  Serial.begin(9600);
  pinMode(13, OUTPUT);  //onboard LED
  pinMode(9, OUTPUT);   //motor pwm pin
  pinMode(10, OUTPUT);  //motor direction pin
  pinMode(2, INPUT);    //encoder
  pinMode(3, INPUT);    //encoder
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
      dir = Serial.read();
      digitalWrite(10, dir);
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

