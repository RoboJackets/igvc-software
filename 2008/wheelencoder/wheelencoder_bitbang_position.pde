#define SPI_SS 10
#define SPI_CLK 11
#define SPI_MISO 12


void setup(){
	Serial.begin(19200);

	pinMode(SPI_SS, OUTPUT);
	pinMode(SPI_CLK, OUTPUT);
	pinMode(SPI_MISO, INPUT);

	digitalWrite(SPI_SS, HIGH);
	digitalWrite(SPI_CLK, LOW);
}

void loop(){
//start main code
unsigned int datatemp;

	digitalWrite(SPI_SS, LOW);
	delayMicroseconds(100);//delay 100us before starting clock

		for(int i = 0; i < 16; i++){
			digitalWrite(SPI_CLK, HIGH);
			delayMicroseconds(10);//delay of 10us before data is sent

			datatemp |= digitalRead(SPI_MISO);
			datatemp <<= 1;
			
			digitalWrite(SPI_CLK, LOW);
			delayMicroseconds(50);//wait the appropriate amount to make this a 2khz signal (or shorter, encoder take 1khz through 50khz)
		}
	digitalWrite(SPI_SS, HIGH);
	delayMicroseconds(50);//the last bit is held for 50us
        delayMicroseconds(1000);//data only refreshed every 1 ms

	unsigned int dataout = (((datatemp >> 13) & 1) << 8) | (((datatemp >> 12) & 1) << 7) | (((datatemp >> 11) & 1) << 6) | (((datatemp >> 10) & 1) << 5) | (((datatemp >> 9) & 1) << 4) | (((datatemp >> 8) & 1) << 3) | (((datatemp >> 2) & 2) << 2) | (((datatemp >> 1) & 1) << 1) | (datatemp & 1);

	Serial.print("rec: ");
	Serial.println(dataout, DEC);
}

