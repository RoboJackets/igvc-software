
const int SafetyLightPin = 2;

bool safetyIsBlinking = false;
long lastSafetyChange = millis();
bool safetyIsOn = true;

void setup()
{
  Serial.begin(9600);
  pinMode(SafetyLightPin, OUTPUT);
  
  digitalWrite(SafetyLightPin, HIGH);
}

void loop()
{
  if(Serial.available() > 0)
  {
    char light = Serial.read();
    switch(light)
    {
      case 'S': // for Safety
      {
        while(Serial.available() == 0);
        char state = Serial.read();
        if(state == 'B') // for Blink
          safetyIsBlinking = true;
        if(state == 'S') // for Solid
          safetyIsBlinking = false;
        break;
      }
      case 'U': // for Underglow
      {
        break;
      }
    }
  }
  
  if(safetyIsBlinking)
  {
    if(millis() - lastSafetyChange >= 250)
    {
      safetyIsOn = !safetyIsOn;
      lastSafetyChange = millis();
    }
    digitalWrite(SafetyLightPin, safetyIsOn);
  }
  else
  {
    digitalWrite(SafetyLightPin, HIGH);
  }
}
