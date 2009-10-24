#include "WProgram.h"

int ledPin =  12;    // LED connected to digital pin 13
int triggerPin =  2;    // LED connected to digital pin 13
int dataPin =  6;    // LED connected to digital pin 13
// The setup() method runs once, when the sketch starts

void setup()   {                
  // initialize the digital pin as an output:
  pinMode(ledPin, OUTPUT);     
  pinMode(triggerPin, OUTPUT); 
  pinMode(dataPin, INPUT);
  //digitalWrite(dataPin, HIGH);
  Serial.begin(115200);  
}

// the loop() method runs over and over again,
// as long as the Arduino has power
int oldv1=0;
unsigned long int oldt1=0;
unsigned long int runs=0;

void loop()                     
{
  runs++;
  int i;
  int bitv[16];
  int ibuf=0;
  unsigned int raw=0;
  int list[]={0,1,2,8,9 ,10,11,12,13};
  int val=0;
  int err;
  unsigned long int end;
  noInterrupts();
  digitalWrite(triggerPin, LOW);    // set the LED off
  delayMicroseconds(200);                  // wait for a second
  
  raw=0;
  val=0;
  
  for(i=15;i>=0;i--){
    digitalWrite(ledPin, HIGH);
    delayMicroseconds(100);
    bitv[i]=digitalRead(6);
    raw=raw|(bitv[i]<<i);
    digitalWrite(ledPin, LOW);
    delayMicroseconds(100);
  }
//  digitalWrite(triggerPin, HIGH);
//  delayMicroseconds(5000);
    delayMicroseconds(100);
    digitalWrite(triggerPin, HIGH);
//    delayMicroseconds(1450);
  
#if 1
  //extract value
  for(i=0;i<9;i++){
    val=val|(bitv[list[i]]<<i);
  }
  end=micros();
  //for(i=0;i<16;i++){
  //interrupts();
  //Serial.println(raw);
  err=(raw==0);
  if( (val!= oldv1)&!(err)){
    //print dv
    Serial.print("v:");
    Serial.print((val>>(8))&15,HEX);
    Serial.print((val>>(4))&15,HEX);
    Serial.println((val)&15,HEX);
    Serial.print("t:");
    Serial.print((runs>>(28))&15,HEX);
    Serial.print((runs>>(24))&15,HEX);
    Serial.print((runs>>(20))&15,HEX);
    Serial.print((runs>>(16))&15,HEX);
    Serial.print((runs>>(12))&15,HEX);
    Serial.print((runs>>(8))&15,HEX);
    Serial.print((runs>>(4))&15,HEX);
    Serial.println((runs)&15,HEX);
    /*Serial.print(ibuf=val%15,HEX);
    Serial.println(val&0b11111111,HEX);
    Serial.println(val%16+1000);*/
    oldt1=runs;
    oldv1=val;
  }
  else{
    delayMicroseconds(1450);
  }
  if (!err){
    oldv1=val;
  }
#endif

  //while(micros()-end<1000);
  /*for(i=0;i<16;i++){
    Serial.print(bitv[i]);
  }
  Serial.println();*/
}

//Die on pure virtual call
extern "C" void __cxa_pure_virtual()
{
	for(;;)
	{

	}
}

int main(void)
{
	init();

	setup();
    
	for (;;)
		loop();
        
	return 0;
}

