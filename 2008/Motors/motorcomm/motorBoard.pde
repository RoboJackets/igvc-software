#define hallLeftMotorPin 0
#define hallRightMotorPin 1
#define hallLogicPin 2
#define hardEStopPin 13
#define leftMotorSpeedPin 5
#define rightMotorSpeedPin 6
#define leftMotorDirPin 7
#define rightMotorDirPin 8

#define varLeftMotorSpeed 0
#define varRightMotorSpeed 1
#define varEStop 2

unsigned char leftMotorSpeed;
unsigned char rightMotorSpeed;
unsigned char softEStop;
unsigned char hardEStop;
unsigned char pathNav;
unsigned char autoMan;
unsigned char logicBatteryV;
unsigned char motorBatteryV;



int hallReadings[2];
unsigned long hallTimestamps[2];

void setup()
{
  Serial.begin(9600);	// opens serial port, sets data rate to 9600 bps
  pinMode(hardEStopPin, INPUT);
  pinMode(leftMotorDirPin, OUTPUT);
  pinMode(rightMotorDirPin, OUTPUT);
  leftMotorSpeed=127;
  rightMotorSpeed=127;
  softEStop=0;
  hardEStop=0;
  autoMan=0;
  pathNav=0;
}

void loop()
{
  readHallSensors();
  hardEStop=digitalRead(hardEStopPin);
  checkSerialBuffer();    //look for commands from the laptop
  setMotorSpeeds();
}

void setMotorSpeeds()
{
//  unsigned char leftMotorSpeed;
//  unsigned char rightMotorSpeed;
unsigned char tempSpeed;
  if (leftMotorSpeed>127)
  {
    tempSpeed=leftMotorSpeed-127;
    analogWrite(leftMotorSpeedPin, tempSpeed);
    digitalWrite(leftMotorDirPin, HIGH);
  }
  else
  {
    analogWrite(leftMotorSpeedPin,leftMotorSpeed);
    digitalWrite(leftMotorDirPin, LOW);
  }
  if (rightMotorSpeed>127)
  {
    tempSpeed=rightMotorSpeed-127;
    analogWrite(rightMotorSpeedPin, tempSpeed);
    digitalWrite(rightMotorDirPin, HIGH);
  }
  else
  {
    analogWrite(rightMotorSpeedPin,rightMotorSpeed);
    digitalWrite(rightMotorDirPin, LOW);
  }
}

void readHallSensors()
{
  hallReadings[0]=analogRead(hallLeftMotorPin);    //read hall sensor data
  hallTimestamps[0]=millis();                      //timestamp it
  hallReadings[1]=analogRead(hallRightMotorPin);
  hallTimestamps[1]=millis();
}

void checkSerialBuffer()
{
  while (Serial.available()>0)    //keep parsing the data until buffer is empty
  {
    unsigned char incomingByte;
    incomingByte=(unsigned char)Serial.read();    //read one byte of data from the buffer
    
	switch(incomingByte)
    {
      case 'r':                //computer wants to read all data from arduino
        //Serial.print(hardEStop, BYTE);
        //Serial.print(softEStop, BYTE);
        //Serial.print(autoMan, BYTE);
        //Serial.print(pathNav, BYTE);
        //Serial.print(hallReadings[0], BYTE);
        //Serial.print(hallReadings[1], BYTE);
        //Serial.print(logicBatteryV, BYTE);
        //Serial.print(motorBatteryV, BYTE);
        Serial.print((unsigned long)65,BYTE);
        Serial.print((unsigned long)66,BYTE);
        Serial.print((unsigned long)67,BYTE);
        Serial.print((unsigned long)68,BYTE);
        Serial.print((unsigned long)69,BYTE);
        Serial.print((unsigned long)70,BYTE);
        Serial.print((unsigned long)71,BYTE);
        Serial.print((unsigned long)72,BYTE);
        Serial.print((unsigned long)73,BYTE);
        Serial.print((unsigned long)74,BYTE);
        Serial.print((unsigned long)75,BYTE);
        
        break;
      case 'w':                //computer is going to write to arduino variable
        unsigned char value;
        incomingByte=(unsigned char)Serial.read();    //incomingByte=Variable number to be written to
        value=(unsigned char)Serial.read();
        writeVariable(incomingByte,value);
        break;
    }
  }
}

unsigned long getSerialValue()
{
  
}

void writeVariable(int variableNumber, unsigned long value)
{
  switch (variableNumber)
  {
    case 0:
      leftMotorSpeed=value;
      break;
    case 1:
      rightMotorSpeed=value;
      break;
    case 2:
      softEStop=value;
      break;
  }
}



