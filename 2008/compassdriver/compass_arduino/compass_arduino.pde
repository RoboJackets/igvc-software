//need to decide how to instruct compass of config, and need to set DATA_LENGTH approp.

//maybe reload config every time arduino starts?

//how does the world want to request and get the data off the arduino?

#include compassTypes.h
#include compassFunctions.h

#define DATA_LENGTH ??
#define DATA_FRAME_COUNT ??

int SPI_SS = 10;
int SPI_CLK = 11;
int SPI_MISO = 12;
int SPI_MOSI = 13;
int SPI_SYNC = 8;//this clears the compass's comm buffer

void setup(){
	Serial.begin(9600);

	pinMode(SPI_SS, OUTPUT);
	pinMode(SPI_CLK, OUTPUT);
	pinMode(SPI_MOSI, OUTPUT);
	pinMode(SPI_MISO, INPUT);
	pinMode(SPI_SYNC, OUTPUT);

	digitalWrite(SPI_SS, LOW);
	digitalWrite(SPI_CLK, LOW);
	digitalWrite(SPI_MOSI, LOW);
	digitalWrite(SPI_SYNC, HIGH);
	delayMicroseconds(100);
	digitalWrite(SPI_SYNC, LOW);
}

void loop(){
	getCommand();
}


int getCommand(){
	while(Serial.available < 1){}//wait here while no data

	incomingByte = Serial.read();

	switch(incomingByte){
		case 'a':
			getmodinfo();
			break;
		case 'b':
			//setData();
			//incomingByte = Serial.read();
			//change of mind -- lets just hardcode the setup -- parsing is too much of a pain
			break;
		case 'c':
			getData();
			break;
		case 'd':
			setConfig();//again, hardcode
			break;
		case 'e':
			startCal();		
			break;
		case 'f':
			stopCal();
			break;
		case 'g':
			getCalData();
			break;
		case 'h':
			setDataComponents();//hardcode
			break;
		default:
			//explode.
			break;
	}
	
}


