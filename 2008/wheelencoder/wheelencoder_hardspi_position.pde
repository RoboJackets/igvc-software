
#define SPI_MOSI 11//MOSI
#define SPI_MISO 12//MISO 
#define SPI_CLK	13//sck
#define SPI_SS	10//ss

void setup(){

	Serial.begin(19200);

	pinMode(SPI_MOSI, OUTPUT);
	pinMode(SPI_MISO, INPUT);
	pinMode(SPI_CLK, OUTPUT);
	pinMode(SPI_SS, OUTPUT);
	digitalWrite(SPI_SS, HIGH); //disable device 

	//the encoder sends at a max of 50khz, arduino at default clock runs spi at minimum 250khz	
	//set the system clock prescaler to 8 so spi_clock ~ 40khz
	CLKPR = (1<<CLKPCE);
	CLKPR = (1<<CLKPS1)|(1<<CLKPS0);

	SPSR ^= ~(1);//disable double speed
	
	SPCR = 0;
	SPCR = (1<<SPE) | (1<<MSTR) | (1<<CPHA) | (1<<SPR1) | (1<<SPR0);//enable spi, set master, set sample clock on trailing edge, slowest speed (250khz->40khz); (implicit: clock idle low, MSB first)
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
	byte datatemplow;
	byte datatemphigh;

	digitalWrite(SPI_SS, LOW);
	delayMicroseconds(100);//wais for encoder to be ready
	
	datatemphigh = spi_transfer(0x00);//read and send null
	datatemplow = spi_transfer(0x00);

	digitalWrite(SPI_SS, HIGH);
	delayMicroseconds(50);//wait while wheel encoder holds data
	delayMicroseconds(1000);// wait 1 miliseconds while encoder updates position

	datatemp = (datatemphigh << 8) | (datatemplow);
	unsigned int dataout = (((datatemp >> 13) & 1) << 8) | (((datatemp >> 12) & 1) << 7) | (((datatemp >> 11) & 1) << 6) | (((datatemp >> 10) & 1) << 5) | (((datatemp >> 9) & 1) << 4) | (((datatemp >> 8) & 1) << 3) | (((datatemp >> 2) & 2) << 2) | (((datatemp >> 1) & 1) << 1) | (datatemp & 1);

	//Serial.print("rec: ");
	Serial.println(dataout, DEC);
}
