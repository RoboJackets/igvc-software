
//pin numbers
const int rightDir = 5;
const int rightSpeed = 6;
const int rightDisable = 7;
const int leftDir = 2;
const int leftSpeed = 3;
const int leftDisable = 4;

int dirRight;
int dirLeft;
int byte0;
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
    byte0 = Serial.read();
    if (byte0 == 'S')
    {
      while (Serial.available()<=0){}
      byte1 = Serial.read(); //First Case either W, R, D, T
      delay(200);
      if (byte1 == 'W' || byte1 == 'R' || byte1 == 'D' || byte1 == 'T')
      {
        while (Serial.available()<=0){}
        byte3 = Serial.parseInt(); //Right Direction
        while (Serial.available()<=0){}
        byte4 = Serial.parseInt(); //Right Speed
        while (Serial.available()<=0){}
        byte5 = Serial.parseInt(); //Left Direction
        while (Serial.available()<=0){}
        byte6 = Serial.parseInt(); //Left Speed
      } 
      else 
      {
        return;
      }
    } 
    else 
    {
      return;
    }
  }
  else
  {
    return;
  }
  
  switch (byte1)
  {
    case 'W':
    {
      //moving wheels in a certain direction and speed
      digitalWrite(rightDir, byte3);
      digitalWrite(leftDir, byte5);
      analogWrite(rightSpeed, byte4);
      analogWrite(leftSpeed, byte6);
      break;
    }
    case 'D':
    {
     //Loop for moving a set distance
      break; 
    }
    
    case 'R':
    {
      //Getting info from encoders
      break;
    }
    
    case 'T':
    {
      //Testing the connection to arduinos will send a confirmaton to the computer
      Serial.println("T");
      break;
    }
  }
  
  
}
      
