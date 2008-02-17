int SPI_SS = 10;
int SPI_CLK = 11;
int SPI_MISO = 12;
int SPI_MOSI = 13;

int SPI_SYNC = 14;//this clears the compass's comm buffer

unsigned char command[3] = {0xAA, 0x01, 0x00};

/*
	data_out[0] = 0xAA; //sync_flag
	data_out[1] = 0x01; //get_mod_info
	data_out[2] = 0x00; //terminator
*/


void setup(){

	pinMode(SPI_SS, OUTPUT);
	pinMode(SPI_CLK, OUTPUT);
	pinMode(SPI_MOSI, OUTPUT);
	pinMode(SPI_MISO, INPUT);
	pinMode(SPI_SYNC, OUTPUT);

	digitalWrite(SPI_SS, HIGH);
	digitalWrite(SPI_CLK, LOW);
	digitalWrite(SPI_MOSI, LOW);
	digitalWrite(SPI_SYNC, HIGH);
delayMicroseconds(100);
	digitalWrite(SPI_SYNC, LOW);

}

void loop(){
//start main code
unsigned char datain[11];
	digitalWrite(SPI_SS, LOW);
	//delayMicroseconds(100);//delay 100us before starting clock

//send command
	for(int byte = 0; byte < 3; byte++){
		/*for(int bit = 0; bit < 8; bit++){
			if((command[byte] >> bit) & 1){//this is sending little endian -- lsb first
				digitalWrite(SPI_CLK, HIGH);
				digitalWrite(SPI_MOSI, HIGH);
				delayMicroseconds(50);//wait the appropriate amount to make this a 2khz signal (or shorter, max 3.6864 Mhz)
			}
			else{
				digitalWrite(SPI_CLK, HIGH);
				digitalWrite(SPI_MOSI, LOW);
				delayMicroseconds(50);//wait the appropriate amount to make this a 2khz signal (or shorter, max 3.6864 Mhz)
			}
			digitalWrite(SPI_CLK, LOW);
			delayMicroseconds(50);//wait the appropriate amount to make this a 2khz signal (or shorter, max 3.6864 Mhz)
		}*/

		shiftOut(SPI_MOSI, SPI_CLK, MSBFIRST, command[byte]);//how fast does this run?
	}
//

delayMicroseconds(100);//maybe?

//rec data
unsigned char datatemp[11];
	digitalWrite(SPI_MOSI, LOW);//transmit zero whilst receiving
	for(int byte = 0; byte < 11; byte++){
		for(int bit = 0; bit < 8; bit++){//this is rec little endian
			digitalWrite(SPI_CLK, HIGH);
			//delayMicroseconds(10);//delay of 10us before data is sent

			datatemp[byte] |= digitalRead(SPI_MISO);
			datatemp[byte] <<= 1;
			
			digitalWrite(SPI_CLK, LOW);
			delayMicroseconds(50);//wait the appropriate amount to make this a 2khz signal (or shorter, max 3.6864 Mhz)
		}
	}
	
	digitalWrite(SPI_SS, HIGH);

	

        Serial.begin(9600);
	Serial.print("received: ");
	for(int data = 0; data < 11; data++){
		Serial.print(data, HEX);
		Serial.print("\t");
		Serial.print(data, BYTE);
		Serial.print("\n");
	}
	Serial.print("\n");
        delay(5000);
}
