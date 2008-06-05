#include "SPI.h"

void InitSPI(void) {
	/* set the pin modes */ //TODO: do this with a loop (?)
	pinMode(SPI_MISO, INPUT);
	pinMode(SPI_MOSI, OUTPUT);
	pinMode(SPI_SS_LEFT_MOTOR_ENCODER, OUTPUT);
	pinMode(SPI_SS_RIGHT_MOTOR_ENCODER, OUTPUT);
	pinMode(SPI_CLK, OUTPUT);

	/* De-select all SPI devices */
	digitalWrite(SPI_SS_LEFT_MOTOR_ENCODER, HIGH);
	digitalWrite(SPI_SS_LEFT_MOTOR_ENCODER, HIGH);

	// Can this be removed?
	TCCR1A = 0;//set to counter mode
	//TCCR1B = (1<<CS12)|(1<<CS10);// clk1o/1024 table on pg134
	//TCCR1B = (1<<CS10);// no prescale clk1o/1
	TCCR1B = (1<<CS11)|(1<<CS10);//clkio/64
}

bool SPIReadBytes(void *data, int numBytes, int inputPin, int slaveSelectPin, int clockPin) {
//#ifdef BITBANG_SPI

	/* Select the device */
	digitalWrite(slaveSelectPin, LOW);
	/* delay 100us before starting clock */
	delayMicroseconds(500);

	/* Read each byte */
	for (int i = 0; i < numBytes; i++) {
		data[i] = 0;
		/* Read a single byte */
		for (int j = 0; j < 8; j++) {
			/* Raise the clock */
			digitalWrite(clockPin, HIGH);
			/* Delay of 10us before data is sent */
			delayMicroseconds(10);

			/* Read a single bit */
			data[i] |= digitalRead(inputPin) << (15 - i);

			/* Lower the clock */
			digitalWrite(clockPin, LOW);
			/* Wait to make this a 20khz signal (encoder goes to 50khz) */
			delayMicroseconds(50);
		}
	}
	/* De-select the device */
	digitalWrite(slaveSelectPin, HIGH);

	/* The last bit is held for 50us */
	delayMicroseconds(50);
	/* Data only refreshed every 1 ms 
	 * note: delay(1) is inaccurate for a 1 ms wait, seems to be only accurate to about max 200us
	 */			
	delayMicroseconds(1000);

	return(data);
//#else
//#ifdef HARD_SPI
	
//#else
//	#error "SPIReadInt: SPI mode not set.\n"
//#endif
//#endif
}

bool SPISendBytes(void *data, int numBytes, int inputPin, int slaveSelectPin, int clockPin) {
}

