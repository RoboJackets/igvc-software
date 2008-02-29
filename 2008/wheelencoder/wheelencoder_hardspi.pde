
//the encoder sends at a max of 50khz, what does the arduino run at? it might be 16Mhz/128 ~ 125khz ...




#define SPI_MOSI 11//MOSI
#define SPI_MISO 12//MISO 
#define SPI_CLK	13//sck
#define SPI_SS	10//ss

void setup(){

	Serial.begin(9600);

	pinMode(SPI_MOSI, OUTPUT);
	pinMode(SPI_MISO, INPUT);
	pinMode(SPI_CLK, OUTPUT);
	pinMode(SPI_SS, OUTPUT);
	digitalWrite(SPI_SS, HIGH); //disable device 
	
	CLKPR = (1<<CLKPCE);
	CLKPR = (1<<CLKPS1)|(1<<CLKPS0);//set the system clock slower so spi_clock is 40khz

	SPSR ^= ~(1);//disable double speed
	
	SPCR = 0;
	SPCR = (1<<SPE) | (1<<MSTR) | (1<<CPHA) | (1<<SPR1) | (1<<SPR0);//enable spi, set master, set sample clock on trailing edge, slowest speed (250khz->40khz); (implicit: clock idle low, MSB first)
	//SPCR = (1<<SPE)|(1<<MSTR);

	Serial.begin(9600);	
}

//verbatim from eeprom tutorial
char spi_transfer(volatile char data){
	SPDR = data;	// Start the transmission
	while (!(SPSR & (1<<SPIF))) // Wait the end of the transmission
	{
	};
	return SPDR;// return the received byte
}


void loop(){
	unsigned int datatemp;
	unsigned char datatemplow;
	unsigned char datatemphigh;

	digitalWrite(SPI_SS, LOW);
	delayMicroseconds(100);//wais for encoder to be ready
	
	datatemphigh = spi_transfer(0x00);//read and send null
	datatemplow = spi_transfer(0x00);

	digitalWrite(SPI_SS, HIGH);
	delayMicroseconds(50);//wait while wheel encoder holds data
	delay(1);// wait 1 miliseconds while encoder updates position

	datatemp = (datatemphigh << 8) | (datatemplow);
	unsigned int dataout = (((datatemp >> 13) & 1) << 8) | (((datatemp >> 12) & 1) << 7) | (((datatemp >> 11) & 1) << 6) | (((datatemp >> 10) & 1) << 5) | (((datatemp >> 9) & 1) << 4) | (((datatemp >> 8) & 1) << 3) | (((datatemp >> 2) & 2) << 2) | (((datatemp >> 1) & 1) << 1) | (datatemp & 1);

	//Serial.print("received: ");
	Serial.println(dataout, DEC);
}
