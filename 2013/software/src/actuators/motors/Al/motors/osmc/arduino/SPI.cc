#include "SPI.hpp"

/* Get a single integer. Not multi-thread safe.*/
//TODO: change this to SPIReadBytes
int SPIReadInt(int inputPin, int slaveSelectPin, int clockPin)
{
//#ifdef BITBANG_SPI
  	unsigned int data = 0;

	digitalWrite(slaveSelectPin, LOW);
	delayMicroseconds(500);//delay 100us before starting clock

	for(int i = 0; i < 16; i++){
		digitalWrite(clockPin, HIGH);
		delayMicroseconds(10);//delay of 10us before data is sent

		//int bit = digitalRead(inputPin);
		//Serial.print(bit);
		data |= digitalRead(inputPin) << (15 - i);
		//data |= bit << (15 - i);


		digitalWrite(clockPin, LOW);
		delayMicroseconds(50);//wait to make this a 20khz signal (encoder goes to 50khz)
	}
	digitalWrite(slaveSelectPin, HIGH);

	delayMicroseconds(50);//the last bit is held for 50us
	delayMicroseconds(1000);//data only refreshed every 1 ms -- note: delay(1) is inaccurate for a 1 ms wait, seems to be only accurate to about max 200us

	//Serial.print("\t");

	return(data);
//#else
//#ifdef HARD_SPI
	
//#else
//	#error "SPIReadInt: SPI mode not set.\n"
//#endif
//#endif
}
