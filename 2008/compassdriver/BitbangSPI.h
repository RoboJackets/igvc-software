#ifndef BITBANG_SPI_H
#define BITBANG_SPI_H

#include<unistd.h>
#include<sys/types.h>
#include<sys/mman.h>
#include<stdio.h>
#include<fcntl.h>
#include<string.h>
#include<unistd.h>

/*
//pin 14
#define SPI_CLK (1 << 13)
//pin 10
#define SPI_MISO 9
//#define ~(*dio_in >> 9) & 1//alternate definition, may be simpler reading
//pin 12
#define SPI_MOSI (1 << 11)
//pin 1
#define SPI_SS (1 << 0)
//pin 6
#define SPI_FRAME (1 << 5)
*/


#define SYSCONT_BASE 0xe8000000
#define INPUT_OFFSET 0x4
#define OUTPUT_OFFSET 0x8

//figure out good sleep mechanism
#define PAUSE usleep(100)

typedef unsigned char uint_8;

/*union u8_bits {
	struct{
		uint_8 bit:1;
	};
	uint_8 u8;
};*/

#define spi_ss pin1
#define spi_frame pin6
#define spi_miso pin10
#define spi_mosi pin12
#define spi_clock pin14

struct dio_pins{
	unsigned pin1:1;
	unsigned pin2:1;
	unsigned pin3:1;
	unsigned pin4:1;//always input
	unsigned pin5:1;
	unsigned pin6:1;
	unsigned pin7:1;
	unsigned pin8:1;
	unsigned pin9:1;
	unsigned pin10:1;
	unsigned pin11:1;
	unsigned pin12:1;
	unsigned pin13:1;
	unsigned pin14:1;
	unsigned pin15:1;
	unsigned pin16:1;
};


class BitbangSPI{
	private:
		//volatile unsigned int *dio_in, *dio_out;
		volatile struct dio_pins *dioblock_in;
		volatile struct dio_pins *dioblock_out;
		int fd_dev_mem;

	public:
		BitbangSPI();
		~BitbangSPI();
		int spiSend(uint_8 * data, int size);
		int spiGet(uint_8 * dataresp, int size);
};

BitbangSPI::BitbangSPI(){
	fd_dev_mem = open("/dev/mem", O_RDWR|O_SYNC);
	unsigned char *base;
	//base = mmap(0, getpagesize(), PROT_READ|PROT_WRITE, MAP_SHARED, fd_dev_mem, SYSCONT_BASE);//this worked before without casting as (unsigned char *).  what changed? -- does c++ not auto cast from void? plain C did.
	base = (unsigned char *) mmap(0, getpagesize(), PROT_READ|PROT_WRITE, MAP_SHARED, fd_dev_mem, SYSCONT_BASE);
	
	//dio_in = (base + INPUT_OFFSET);  // dio in block
	//dio_out = (base + OUTPUT_OFFSET);  // dio out block

	dioblock_in = (struct dio_pins *) (base + INPUT_OFFSET);//cast as struct
	dioblock_out = (struct dio_pins *) (base + OUTPUT_OFFSET);


	//init the pins
	dioblock_out->spi_ss = 1;
	dioblock_out->spi_mosi = 0;
	dioblock_out->spi_frame = 0;
	dioblock_out->spi_clock = 0;
	//dio_in = 0;  // init to off -- this probly doesn't do anything
	//dio_out = 0;  // init to off -- do this somewhere else now
}

BitbangSPI::~BitbangSPI(){
	//dio_out = 0;  // set to off -- do this different
	close(fd_dev_mem);
}

int BitbangSPI::spiSend(uint_8 * data, int size) {

	//u8_bits	bitfield;
	//bitfield.u8 = data;
	

	int i = 0;
	int byte = 0;
	//*dio_out = 0;//make sure all off?
	dioblock_out->spi_ss = 0;//pull low to make compass listen
	dioblock_out->spi_clock = 0;//clock low
	PAUSE;
	for(byte = 0;byte < size; byte++){
		for (i=0;i<8;i++){

			dioblock_out->spi_clock = 1;//clock high
			PAUSE;

			if((data[byte] >> i) & 1){//if data bit set (in current byte)
				dioblock_out->spi_clock = 0;//clock low
				dioblock_out->spi_mosi = 1;//set data out high -- this doesn't have to change untill the next time the clock is pulled low
				PAUSE;
			}
			else{
				dioblock_out->spi_mosi = 0;
				dioblock_out->spi_clock = 0;//clock low
				PAUSE;
			}
		}
	}
	//dioblock_out->spi_clock = 0;// the clock should be low
	dioblock_out->spi_ss = 1;//will this make the compass not send untill slave line pulled low again?
	return(0);
}


int BitbangSPI::spiGet(uint_8 * dataresp, int size){
	int i = 0;
	int byte = 0;

	uint_8 data = 0;
	dioblock_out->spi_ss = 0;//pull low to make compass send?
	dioblock_out->spi_clock = 0;//clock low
	PAUSE;

	for(byte = 0;byte < size; byte++){
		for(i=0;i<8;i++){
			dioblock_out->spi_clock = 1;//clock high
			PAUSE;
		
			//data.bits[i] = ~(*dio_in >> SPI_MISO) & 1;//check MISO line
			data |= dioblock_in->spi_miso;
			data <<= 1;//shifting correct?

			dioblock_out->spi_clock = 0;//clock low
			PAUSE;
		}
	dataresp[byte] = data;
	}
	
	//dioblock_out->spi_clock = 0;// the clock should be low
	dioblock_out->spi_ss = 1;//will this make the compass not send untill slave line pulled low again?
	return(0);
}

/**dio_out ^= SPI_SS;//turn on slave select now?
	*dio_out ^= SPI_CLK;
	for(i=0;1<8;1++) {
		*dio_out ^= SPI_CLK;//clock low
		PAUSE;
		if(bitfield.bits(i){
			*dio_out ^= SPI_MOSI;//out
		}
		//when do i turn off the output pin?
		usleep(n);//sleep till clock off
		*dio_out ^= SPI_CLK;//clock off?
		usleep(n);//sleep till next cycle
	}
*/

#endif //BITBANG_SPI_H
