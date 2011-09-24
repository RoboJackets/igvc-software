#include "WProgram.h"

#define encoderPinA 4
#define encoderPinB 6
#define encoderOutputI 5

int counter=0;
int stateA;
int stateB;
int lastStateA=LOW;
int lastStateB=LOW;

void loopTest();

int main()
{
init();
pinMode(encoderPinA,INPUT);
pinMode(encoderPinB,INPUT);
loopTest();
return 0;
}

void loopTest()
{
while(true)
{
stateA=digitalRead(encoderPinA);
stateB=digitalRead(encoderPinB);
if(stateA=!lastStateA && encoderPinB!=lastStateB)
{
counter++;
lastStateA=stateA;
lastStateB=stateB;
}

printf("%d\n",counter);
}
}
