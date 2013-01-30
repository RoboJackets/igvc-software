
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
  
  Serial.begin(9600);
  pinMode(rightDir, OUTPUT);
  pinMode(rightSpeed, OUTPUT);
  pinMode(rightDisable, OUTPUT);
  pinMode(leftDir, OUTPUT);
  pinMode(leftSpeed, OUTPUT);
  pinMode(leftDisable, OUTPUT);
}

void loop()
{
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
          analogWrite(leftSpeed,byte6);
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
      
