
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
	
	SPSR ^= ~(1);//disable double speed
	
	SPCR = 0;
	SPCR = (1<<SPE) | (1<<MSTR) | (1<<CPHA) | (1<<SPR1) | (1<<SPR0);//enable spi, set master, set sample clock on trailing edge, slowest speed (250khz); (implicit: clock idle low, MSB first)	
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
	digitalWrite(SPI_SS, LOW);
	unsigned char data = spi_transfer(0xAA);

	Serial.print("received: ");
	Serial.println(data, DEC);
}
