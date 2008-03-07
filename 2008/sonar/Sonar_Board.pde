#include <Wire.h>
#define readingIndex(addr, highByte) (addr-0xE0+highByte)
// I2C SRF10 or SRF08 Devantech Ultrasonic Ranger Finder 
// by Nicholas Zambetti <http://www.zambetti.com> 
// and James Tichenor <http://www.jamestichenor.net> 
int reading;
int readings[22] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int mode=0;

void setup() 
{ 
  Wire.begin();                // join i2c bus (address optional for master) 
  Serial.begin(9600);          // start serial communication at 9600bps 
} 
 
void loop() 
{ 
  int addr=0;
  readSerial();
  switch (mode)
  {
    case 0:
    break;
    case 4:
    case 1:
      pingSonar(0x00);
      for (addr=0xE0;addr<=0xF4;addr+=2)
      {
        readSonar(addr);
      }
    break;
    case 5:
    case 2:
      for (addr=0xE0;addr<=0xF4;addr+=2)
      {
        pingSonar(addr);
        readSonar(addr);
      }
    break;
    case 6:
    case 3:
      for (addr=0xF4;addr>=0xE0;addr-=2)
      {
        pingSonar(addr);
        readSonar(addr);
      }
    break;
    default:
      pingSonar(mode);
      readSonar(mode);
    break;
  }
  if (mode)
  {
    serialDump();
  }
  if (mode>3)
  {
    mode=0;
  }
}

void readSerial()
{
  if (Serial.available() > 0)
  {
    mode = Serial.read();
    //Serial.println(mode);
  }
}

void pingSonar(int addr)
{
  addr=addr/2;
  //SDA=analog4
  //SCL=analog5
  // step 1: instruct sensor to read echoes 
  Wire.beginTransmission(addr); // transmit to device #112 (0x70) 
                               // the address specified in the datasheet is 224 (0xE0) 
                               // but i2c adressing uses the high 7 bits so it's 112 
  Wire.send(0x00);             // sets register pointer to the command register (0x00)  
  Wire.send(0x51);             // command sensor to measure in "inches" (0x50) 
                               // use 0x51 for centimeters 
                               // use 0x52 for ping microseconds 
  Wire.endTransmission();      // stop transmitting 
 
  // step 2: wait for readings to happen 
  delay(70);                   // datasheet suggests at least 65 milliseconds
}
//------------------------------------------------------------------------------------
void readSonar(int addr)
{
  int realAddr;
  realAddr=addr;
  addr=addr/2;
  // step 3: instruct sensor to return a particular echo reading 
  Wire.beginTransmission(addr); // transmit to device #112 
  Wire.send(0x02);             // sets register pointer to echo #1 register (0x02) 
  Wire.endTransmission();      // stop transmitting 
 
  // step 4: request reading from sensor 
  Wire.requestFrom(addr, 2);    // request 2 bytes from slave device #112 
 
  // step 5: receive reading from sensor 
  if(2 <= Wire.available())    // if two bytes were received 
  { 
    readings[readingIndex(realAddr,0)]=Wire.receive();
    //reading = readings[readingIndex(addr,0)];  // receive high byte (overwrites previous reading)
    readings[readingIndex(realAddr,1)]=Wire.receive(); 
    //reading = reading << 8;    // shift high byte to be high 8 bits 
    //reading |= readings[readingIndex(addr,1)]; // receive low byte as lower 8 bits 
    //Serial.println();   // print the reading 
    //Serial.println(reading);   // print the reading 
    //Serial.println();   // print the reading 
  } 
}

void serialDump()
{
  int i;
  for (i=0;i<22;i++)
  {
    Serial.print((char)readings[i], BYTE);
  }
}
