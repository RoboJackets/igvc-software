
const int rightDir = 5;
const int rightSpeed = 6;
const int rightDisable = 7;
const int leftDir = 2;
const int leftSpeed = 3;
const int leftDisable = 4;
double dist;
int dirRight;
int dirLeft;
int byte1;
int byte2;
int byte3;
int byte4;
int byte5;
int byte6;


void setup()
{
//  TCCR2B = TCCR2B & 0xF8 | 0x03;
//  TCCR0B = TCCR0B & 0xF8 | 0x03;
  Serial.begin(9600);
  pinMode(rightDir, OUTPUT);
  pinMode(rightSpeed, OUTPUT);
  pinMode(rightDisable, OUTPUT);
  pinMode(leftDir, OUTPUT);
  pinMode(leftSpeed, OUTPUT);
  pinMode(leftDisable, OUTPUT);

}

void waitForSerial()
{
  while(Serial.available() == 0) { }
}

void loop()
{
  /*
  waitForSerial();
  byte1 = Serial.read();
  if(byte1 == 'W')
  {
    waitForSerial();
    byte2 = Serial.read();
    if( byte2 == 'S')
    {
      byte3 = Serial.read();
      waitForSerial();
      byte4 = Serial.read();
      waitForSerial();
      byte5 = Serial.read();
      waitForSerial();
      byte6 = Serial.read();
      
      digitalWrite(rightDir, byte3);
      digitalWrite(leftDir, byte5);
      analogWrite(rightSpeed, byte4);
      analogWrite(leftSpeed, byte6);
    }
  }
  */

  if (Serial.available()>0)
  {
    byte1 = Serial.read(); //First Case either W, R, S, T
    delay(200);
    byte2 = Serial.read(); //Second Case only applicable under byte1 == W either S, D
    byte3 = Serial.read(); //Right Direction
    byte4 = Serial.read(); //Right Speed
    byte5 = Serial.read(); //Left Direction
    byte6 = Serial.read(); //Left Speed
    dist = Serial.parseFloat();
  }
  switch (byte1)
  {
    case 'W':
    {
      switch (byte2)
      {
        case 'S':
        {
          //moving both wheels in the same direction and speed
          digitalWrite(rightDir, byte3);
          digitalWrite(leftDir, byte5);
          analogWrite(rightSpeed, byte4);
          analogWrite(leftSpeed, byte6);
          break;
        }
        
        case 'D':
        {
         //Encoder feed back loop for moving a set distance        
         break; 
        } 
      }
    }
    
    case 'R':
    {
      
      break;
    }
    
    case 'S':
    {
      //Stop the motors
      digitalWrite(rightDisable, HIGH);
      digitalWrite(leftDisable, HIGH);
      break;
    }
    
    case 'T':
    {
      //Testing the connection to arduinos will just return a confirmaton to the computer
      Serial.println("T");
    }
  }
  
  
}
      
