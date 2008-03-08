int SPI_SS = 10;
int SPI_CLK = 11;
int SPI_MISO = 12;


void setup(){
	pinMode(SPI_SS, OUTPUT);
	pinMode(SPI_CLK, OUTPUT);
	pinMode(SPI_MISO, INPUT);

	digitalWrite(SPI_SS, HIGH);
	digitalWrite(SPI_CLK, LOW);
	
	TCCR1A = 0;//set to counter mode
	//TCCR1B = (1<<CS12)|(1<<CS10);// clk1o/1024 table on pg134
	TCCR1B = (1<<CS10);// no prescale clk1o/1

	Serial.begin(9600);
}

unsigned int getPostion(){
	unsigned int datatemp;

	digitalWrite(SPI_SS, LOW);
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

	unsigned int dataout = (((datatemp >> 13) & 1) << 8) | (((datatemp >> 12) & 1) << 7) | (((datatemp >> 11) & 1) << 6) | (((datatemp >> 10) & 1) << 5) | (((datatemp >> 9) & 1) << 4) | (((datatemp >> 8) & 1) << 3) | (((datatemp >> 2) & 2) << 2) | (((datatemp >> 1) & 1) << 1) | (datatemp & 1);

	return(dataout);
}

unsigned int getTime(){
	unsigned int time;
	//TCNT1 is a 16 bit timer/counter ~ pg 121
	time = TCNT1L;//how often does this tick? does it tick with clk_cpu
	time |= TCNT1H << 8;
	return(time);
}

void loop(){

	unsigned long t1 = getTime();
	
	unsigned int p1 = getTime();
	unsigned int p2 = getPostion();
      
	unsigned long t2 = getTime();



	//Serial.print("received: ");
//	Serial.println("dp");
//	Serial.println(p2 - p1, DEC);
//	Serial.println("dt");

	float velocity;
	if((t2 > t1) && (p2 > p1)){
  		velocity = ((p2 - p1) * TWO_PI / 512) / ((t2 - t1)/F_CPU);
	}
	else if((t2 < t1) && (p2 > p1)){
  		velocity = ((p2 - p1) * TWO_PI / 512) / (((t2+65536) - t1)/F_CPU);
	}
	else if((t2 > t1) && (p2 < p1)){
  		velocity = (((p2+512) - p1) * TWO_PI / 512) / ((t2 - t1)/F_CPU);
	}
	else if((t2 < t1) && (p2 < p1)){
  		velocity = (((p2+512) - p1) * TWO_PI / 512) / (((t2+65536) - t1)/F_CPU);
  		
	}
	Serial.println(velocity, DEC);
}
