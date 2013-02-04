
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
  /*
  digitalWrite(rightDir, 1);
  digitalWrite(leftDir, 1);
  
  //analogWrite(rightSpeed, 255);
  //analogWrite(leftSpeed, 255);
  
  delay(3000);
  

  
  for (int i = 255; i>0;i-=1)
  {
    analogWrite(rightSpeed, i);
    analogWrite(leftSpeed, i);
    delay(50);
  }
  
for (int i = 0; i<255;i+=1)
  {
    analogWrite(rightSpeed, i);
    analogWrite(leftSpeed, i);
    delay(50);
  }
  
    analogWrite(rightSpeed, 255);
    analogWrite(leftSpeed, 255);
  
  
  
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
    byte1 = Serial.read(); //First Case either W, R, D, T
    delay(200);
    if (byte1 == 'W' || byte1 == 'R' || byte1 == 'D' || byte1 == 'T')
    {
      byte3 = Serial.read(); //Right Direction
      byte4 = Serial.read(); //Right Speed
      byte5 = Serial.read(); //Left Direction
      byte6 = Serial.read(); //Left Speed
    }
   // dist = Serial.parseFloat();
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
    }
  }
  
  
}
      
