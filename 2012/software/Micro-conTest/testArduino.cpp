#include "WProgram.h"

void testDigital(int p);
void testAnalog(int p);

int main()
{
int pin=0;
int analogPins[]={3,5,6,9,10,11};
while(pin<14)
{
testDigital(pin);
}
pin=0;
while(pin<6)
{
testAnalog(analogPins[pin]);
}

return 0;
}

void testDigital(int pin)
{
pinMode(pin,OUTPUT);
digitalWrite(pin,HIGH);
delay(5000);
digitalWrite(pin,LOW);
pin++;
}

void testAnalog(int pin)
{
analogWrite(pin,200);
delay(5000);
analogWrite(pin,0);
pin++;
}

