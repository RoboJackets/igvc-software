#ifndef BITBANG_SPI_H
#define BITBANG_SPI_H

//some of this came from some website i now lost, and is modeled after TS's tempSensor.c

#include<unistd.h>
#include<sys/types.h>
#include<sys/mman.h>
#include<stdio.h>
#include<fcntl.h>
#include<string.h>
#include<unistd.h>

//debug opts
#define PRINT_TO_SCREEN_Rx
#define PRINT_TO_SCREEN_Tx





#define SYSCONT_BASE 0xe8000000
#define INPUT_OFFSET 0x4
#define OUTPUT_OFFSET 0x8

//figure out good sleep mechanism -- ts-7800 has a microsecond clock availble at 0xE8000040
#define PAUSE usleep(100)

//this is defined in another header -- is that bad?
typedef unsigned char uint_8;
/*
#define spi_ss pin1
#define spi_frame pin6
#define spi_miso pin10
#define spi_mosi pin12
#define spi_clock pin14
*/

#define spi_ss_on 1 << 0
#define spi_frame_on 1 << 6
#define spi_miso_on 1 << 10
#define spi_mosi_on 1 << 12
#define spi_clock_on 1 << 14

#define spi_ss_off ~(1 << 0)
#define spi_frame_off ~(1 << 6)
#define spi_miso_off ~(1 << 10)
#define spi_mosi_off ~(1 << 12)
#define spi_clock_off ~(1 << 14)



union dio_pins{
	struct {
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
		unsigned:14;//lcd pins
		unsigned greenled:1;
		unsigned:1;//temp sensor select
	};
	unsigned bytes;
};

class BitbangSPI{
	public:
		//volatile unsigned int *dio_in, *dio_out;
		volatile dio_pins *dioblock_in;
		volatile dio_pins *dioblock_out;
		int fd_dev_mem;

		BitbangSPI();
		~BitbangSPI();
		int spiSend(uint_8 * data, int size);
		int spiGet(uint_8 * dataresp, int size);
		uint_8 lastcommand;
};

BitbangSPI::BitbangSPI(){
	fd_dev_mem = open("/dev/mem", O_RDWR|O_SYNC);
		if(fd_dev_mem == -1) { 
			printf("failed to open /dev/mem\n");
		}
	unsigned char *base;
	//base = mmap(0, getpagesize(), PROT_READ|PROT_WRITE, MAP_SHARED, fd_dev_mem, SYSCONT_BASE);//this worked before without casting as (unsigned char *).  what changed? -- does c++ not auto cast from void? plain C did.
	base = (unsigned char *) mmap(0, getpagesize(), PROT_READ|PROT_WRITE, MAP_SHARED, fd_dev_mem, SYSCONT_BASE);
	
	//dio_in = (base + INPUT_OFFSET);  // dio in block
	//dio_out = (base + OUTPUT_OFFSET);  // dio out block

	dioblock_in = (dio_pins *) (base + INPUT_OFFSET);//cast as struct
	dioblock_out = (dio_pins *) (base + OUTPUT_OFFSET);


	//init the pins
	dioblock_out->bytes |= spi_ss_on;
	dioblock_out->bytes &= spi_mosi_off;
	dioblock_out->bytes &= spi_frame_off;
	dioblock_out->bytes &= spi_clock_off;
	//dio_in = 0;  // init to off -- this probly doesn't do anything
	//dio_out = 0;  // init to off -- do this somewhere else now
}

BitbangSPI::~BitbangSPI(){
	//dio_out = 0;  // set to off -- do this different
	printf("Bitbang spi destructor\n");
	close(fd_dev_mem);
}

#endif //BITBANG_SPI_H
