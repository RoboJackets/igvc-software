/*
/////////////////////////////////
Htachi HM55B Compass
parallax (#)

AUTHOR:   kiilo kiilo@kiilo.org
License:  http://creativecommons.org/licenses/by-nc-sa/2.5/ch/

http://parallax.com/Store/Microcontrollers/BASICStampModules/tabid/134/txtSearch/hm55b/List/1/ProductID/98/Default.aspx?SortField=ProductName%2cProductName
http://sage.medienkunst.ch/tiki-index.php?page=HowTo_Arduino_Parallax_HM55B_Kompass
http://arduino.cc/playground/HM55B

/////////////////////////////////
*/
#include <math.h> // (no semicolon)
#include "WProgram.h"
//// VARS
const int CLK_pin = 8;
const int EN_pin = 9;
const int DIO_pin = 10;

int X_Data = 0;
int Y_Data = 0;
int angle;

//// FUNCTIONS

void ShiftOut(int Value, int BitsCount) {
  for(int i = BitsCount; i >= 0; i--) {
    digitalWrite(CLK_pin, LOW);
    if ((Value & 1 << i) == ( 1 << i)) {
      digitalWrite(DIO_pin, HIGH);
      //Serial.print("1");
    }
    else {
      digitalWrite(DIO_pin, LOW);
      //Serial.print("0");
    }
    digitalWrite(CLK_pin, HIGH);
    delayMicroseconds(1);
  }
//Serial.print(" ");
}

int ShiftIn(int BitsCount) {
  int ShiftIn_result;
    ShiftIn_result = 0;
    pinMode(DIO_pin, INPUT);
    for(int i = BitsCount; i >= 0; i--) {
      digitalWrite(CLK_pin, HIGH);
      delayMicroseconds(1);
      if (digitalRead(DIO_pin) == HIGH) {
        ShiftIn_result = (ShiftIn_result << 1) + 1; 
        //Serial.print("x");
      }
      else {
        ShiftIn_result = (ShiftIn_result << 1) + 0;
        //Serial.print("_");
      }
      digitalWrite(CLK_pin, LOW);
      delayMicroseconds(1);
    }
  //Serial.print(":");

// below is difficult to understand:
// if bit 11 is Set the value is negative
// the representation of negative values you
// have to add B11111000 in the upper Byte of
// the integer.
// see: http://en.wikipedia.org/wiki/Two%27s_complement
  if ((ShiftIn_result & 1 << 11) == 1 << 11) {
    ShiftIn_result = (B11111000 << 8) | ShiftIn_result; 
  }


  return ShiftIn_result;
}

void HM55B_Reset() {
  pinMode(DIO_pin, OUTPUT);
  digitalWrite(EN_pin, LOW);
  ShiftOut(B0000, 3);
  digitalWrite(EN_pin, HIGH);
}

void HM55B_StartMeasurementCommand() {
  pinMode(DIO_pin, OUTPUT);
  digitalWrite(EN_pin, LOW);
  ShiftOut(B1000, 3);
  digitalWrite(EN_pin, HIGH);
}

int HM55B_ReadCommand() {
  int result = 0;
  pinMode(DIO_pin, OUTPUT);
  digitalWrite(EN_pin, LOW);
  ShiftOut(B1100, 3);
  result = ShiftIn(3);
  return result;
}

int main()
{
	setup();
	loop();
}

void setup() {
  Serial.begin(115200);
  pinMode(EN_pin, OUTPUT);
  pinMode(CLK_pin, OUTPUT);
  pinMode(DIO_pin, INPUT);

  HM55B_Reset();
}

void loop() {
  HM55B_StartMeasurementCommand(); // necessary!!
  delay(40); // the data is 40ms later ready
  Serial.print(HM55B_ReadCommand()); // read data and print Status
  Serial.print(" ");  
  X_Data = ShiftIn(11); // Field strength in X
  Y_Data = ShiftIn(11); // and Y direction
  Serial.print(X_Data); // print X strength
  Serial.print(" ");
  Serial.print(Y_Data); // print Y strength
  Serial.print(" ");
  digitalWrite(EN_pin, HIGH); // ok deselect chip
  angle = 180 * (atan2(-1 * Y_Data , X_Data) / M_PI); // angle is atan( -y/x) !!!
  Serial.print(angle); // print angle
  Serial.println("");

}

