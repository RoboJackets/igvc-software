#include "Wire.h"
void setup(){
	Wire.begin();
	Wire.beginTransmission(0x00);
	Wire.send(0x00);
	Wire.send(0xA0);
	Wire.endTransmission();
	Wire.beginTransmission(0x00);
	Wire.send(0x00);
	Wire.send(0xAA);
	Wire.endTransmission();
	Wire.beginTransmission(0x00);
	Wire.send(0x00);
	Wire.send(0xA5);
	Wire.endTransmission();
	Wire.beginTransmission(0x00);
	Wire.send(0x00);
	Wire.send(0xF8); //<<Change this to the desired address and load into Arduino memory
	Wire.endTransmission();
}
void loop(){


}
