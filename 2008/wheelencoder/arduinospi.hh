#ifndef ARDUINOSPI_HH
#define ARDUINOSPI_HH

//for photocraft SR12-512N/5

include "cstdlib"
include "unistd.h"

//bitbang spi on arduino

#define  WAIT_100u usleep(100)
#define WAIT_2k usleep(50)
#define WAIT_10u usleep(10)
#define WAIT_50u usleep(50)

#define SPI_MISO 1
#define SPI_CLK 3
#define SPI_SS 4

#define PRINT_TO_SCREEN_Rx

typedef union{
	struct{
		short int:2;
		short int:d8
		short int:d7
		short int:d6
		short int:d5
		short int:d4
		short int:d3
		short int:5;
		short int:d2;
		short int:d1;
		short int:d0;
	};
	short int sint_16;
} encoderdata;

class ARDUINO_SPI{
	private:
		

	public:
		ARDUINO_SPI(void);
		~ARDUINO_SPI(void);

		short int spiGet(void);
}


ARDUINO_SPI::ARDUINO_SPI(void){
	pinmode(SPI_SS, OUTPUT);
	pinmode(SPI_CLK, OUTPUT);
	pinmode(SPI_MISO, INPUT;

	digitalWrite(SPI_SS, HIGH);
	digitalWrite(SPI_CLK, LOW);
}

short int ARDUINO_SPI::spiGet(void){
//	uint_8 data = 0;
	encoderdata datatemp;
	digitalWrite(SPI_SS, LOW);
	WAIT_100u;//delay 100us before starting clock

		for(int i = 0; i < 16; i++){
			digitalWrite(SPI_CLK, HIGH);
			WAIT_10u;//delay of 10us before data is sent

			datatemp.sint_16 |= digitalRead(SPI_MISO);
			datatemp.sint_16 <<= 1;
			
			digitalWrite(SPI_CLK, LOW);
			WAIT_2k;//wait the appropriate amount to make this a 2khz signal (or shorter, encoder take 1khz through 50khz)
		}
	digitalWrite(SPI_SS, HIGH);
	WAIT_50u;//the last bit is held for 50us

	short int dataout = (datatemp.d8 << 8) | (datatemp.d7 << 7) | (datatemp.d6 << 6) | (datatemp.d5 << 5) | (datatemp.d4 << 4) | (datatemp.d3 << 3) | (datatemp.d2 << 2) | (datatemp.d1 << 1) | (datatemp.d0)

	#ifdef PRINT_TO_SCREEN_Rx
	printf("received:\n");
	for(int i = 0; i < length; i++){
		printf("\tpacket %i = %X\n",i,dataresp[i]);
	}
	#endif
	
}

#endif //ARDUINOSPI_HH
