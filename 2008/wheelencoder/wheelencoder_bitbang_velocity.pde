#include <stdio.h>

#define RADIUS .005
#define COUNTER_SCALER (64)
#define COUNTER_RATE (F_CPU/COUNTER_SCALER)
#define RAD_ENCODERTICK (TWO_PI/512)

#define SPI_SS 10
#define SPI_CLK 11
#define SPI_MISO 12

float lastp1, lastp2;


void setup(){
	pinMode(SPI_SS, OUTPUT);
	pinMode(SPI_CLK, OUTPUT);
	pinMode(SPI_MISO, INPUT);

	digitalWrite(SPI_SS, HIGH);
	digitalWrite(SPI_CLK, LOW);
	
	TCCR1A = 0;//set to counter mode
	//TCCR1B = (1<<CS12)|(1<<CS10);// clk1o/1024 table on pg134
	//TCCR1B = (1<<CS10);// no prescale clk1o/1
	TCCR1B = (1<<CS11)|(1<<CS10);//clkio/64


	Serial.begin(9600);

float lastp1 = 0;
float lastp2 = 0;
}

unsigned int getTime(){
	unsigned int time;
	//TCNT1 is a 16 bit timer/counter ~ pg 121
	time = TCNT1L;//how often does this tick? does it tick with clk_cpu
	time |= TCNT1H << 8;
	return(time);
}

void getPosition(unsigned int * time, unsigned int * position){
	unsigned int datatemp;

	digitalWrite(SPI_SS, LOW);
	*time = getTime();
	delayMicroseconds(100);//delay 100us before starting clock

	for(int i = 0; i < 16; i++){
		digitalWrite(SPI_CLK, HIGH);
		delayMicroseconds(10);//delay of 10us before data is sent

		datatemp |= digitalRead(SPI_MISO);
		datatemp <<= 1;
			
		digitalWrite(SPI_CLK, LOW);
		delayMicroseconds(50);//wait the appropriate amount to make this a 2khz signal (or shorter, encoder does 1khz through 50khz)
	}
	digitalWrite(SPI_SS, HIGH);
	
	delayMicroseconds(50);//the last bit is held for 50us
	delay(1);//data only refreshed every 1 ms

	*position = (((datatemp >> 13) & 1) << 8) | (((datatemp >> 12) & 1) << 7) | (((datatemp >> 11) & 1) << 6) | (((datatemp >> 10) & 1) << 5) | (((datatemp >> 9) & 1) << 4) | (((datatemp >> 8) & 1) << 3) | (((datatemp >> 2) & 2) << 2) | (((datatemp >> 1) & 1) << 1) | (datatemp & 1);
}


void loop(){
	unsigned int t1,t2,p1,p2;	

//do{
 
	getPosition(&t1, &p1);
	//delay(10);
	getPosition(&t2, &p2);
 
  /*
   if(( p1 == 0) && (lastp1 != 0)){//this is supposed to fix the random zero reawds but seems to cause other problems 
    p1 = lastp1;
   }
   if(( p2 == 0) && (lastp2 != 0)){
    p2 = lastp2;
   }
*/
//lastp1 = p1;
//lastp2 = p2;
//}while( ((p1 == lastp1) && (p2 != lastp2)) || ((p2 == lastp2) && (p1 != lastp1)) );

	unsigned int dt;
        int dp;
	if( (p2 > p1) && ( (p1+512) - p2 < 100) ){
                dp = -1*((p1 + 512) - p2);
        }
        else if(p2 >= p1){
  		dp = p2 - p1;
	}
	else if( (p2 < p1) && ( (p1 - p2) > 300) ){
  		dp = ( p2 + 512 ) - p1;
	}
	else if((p2 < p1)){
  		dp = p2 - p1;
	}
	

        if(t2 >= t1){
		dt = t2 - t1;
	}
	else if(t2 < t1){
		dt = ( t2 + 65536 ) - t1;
  	}

	float velocity = ( (float)dp) / ( (float)dt) * RAD_ENCODERTICK * COUNTER_RATE;

	char strbuff[32];
	Serial.print("ang vel(rad/s): ");
	Serial.print(dtostrf(velocity, 2, 3, strbuff));
	Serial.print("\tp1: ");
	Serial.print(p1, DEC);
	Serial.print("\tp2: ");
	Serial.print(p2, DEC);
        Serial.print("\tdp: ");
        Serial.print(dp, DEC);
	Serial.print("\tt1: ");
	Serial.print(t1, DEC);
	Serial.print("\tt2: ");
	Serial.print(t2, DEC);
        Serial.print("\tdt: ");
        Serial.print(dt, DEC);
	Serial.print("\tdt(s): ");
	Serial.print(dtostrf( (float)dt / COUNTER_RATE, 2, 5, strbuff));
	Serial.print("\n");
}

