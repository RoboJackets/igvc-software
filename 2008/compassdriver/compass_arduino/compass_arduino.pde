typedef unsigned char uint_8

int SPI_SS = 10;
int SPI_CLK = 11;
int SPI_MISO = 12;
int SPI_MOSI = 13;

int SPI_SYNC = 8;//this clears the compass's comm buffer

#include compassTypes.h

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

void compassSend(uint_8 * dataout, int length){
	for(int frame = 0; frame < length; frame++){
		for(int bit = 0; bit < 8; bit++){
			digitalWrite(SPI_CLK, HIGH);
			if((command[frame] >> bit) & 1){//this is sending little endian -- lsb first
				digitalWrite(SPI_MOSI, HIGH);
			}
			else{
				digitalWrite(SPI_MOSI, LOW);
			}
			delayMicroseconds(50);//wait the appropriate amount to make this a 2khz signal (or shorter, max 3.6864 Mhz)
			digitalWrite(SPI_CLK, LOW);
			delayMicroseconds(50);//wait the appropriate amount to make this a 2khz signal (or shorter, max 3.6864 Mhz)
		}
		//shiftOut(SPI_MOSI, SPI_CLK, LSBFIRST, command[frame]);//how fast does this run?
	}
}

void compassGet(uint_8 * datain, int length){
	digitalWrite(SPI_MOSI, LOW);//transmit zero whilst receiving
	for(int frame = 0; frame < length; frame++){
		for(int bit = 0; bit < 8; bit++){//this is rec little endian
			digitalWrite(SPI_CLK, HIGH);
			//delayMicroseconds(10);//delay of 10us before data is sent

			datatemp[frame] |= digitalRead(SPI_MISO);
			datatemp[frame] <<= 1;
			
			digitalWrite(SPI_CLK, LOW);
			delayMicroseconds(50);//wait the appropriate amount to make this a 2khz signal (or shorter, max 3.6864 Mhz)
		}
	}
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
		case 'c':
			//getData();
		default:
			//
	}

/*
	if (incomingByte == 'a'){//need to figure out an interface
		getmodinfo();
	}
	else if (incomingByte == 'b') {
		//setdata();
	}
	else if (incomingByte == 'c') {
		//getdata();
	}
	
*/
	
}


void getmodinfo(){
	uint_8 command[3] = {sync_flag, get_mod_info, terminator};
	uint_8 answer[11];

	compassSend(data, 3);

	compassGet(answer, 11);

	for(int i = 2; i < 6; i++){
		Serial.print(answer[i], BYTE);
	}
		Serial.print('\n');
	for(int i = 6; i < 10; i++){
		Serial.print(answer[i], BYTE);
	}
	
	//return();
}
