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
//#define NO_SPI
//#define FAKE_RESP

/*
//pin 14
#define SPI_CLK (1 << 13)
//pin 10
#define SPI_MISO 9
//#define ~(*dio_in >> 9) & 1//alternate definition, may be simpler reading later
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

//figure out good sleep mechanism -- ts-7800 has a microsecond clock availble at 0xE8000040
#define PAUSE usleep(100)

//this is defined in another header -- is that bad?
typedef unsigned char uint_8;

#define spi_ss pin1
#define spi_frame pin6
#define spi_miso pin10
#define spi_mosi pin12
#define spi_clock pin14

struct dio_pins{
	unsigned pin1:1;
	unsigned pin2:1;
	unsigned pin3:1;
	unsigned pin4:1;//always input?
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
	unsigned:14;
	unsigned greenled:1;
	unsigned:1;
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
		uint_8 lastcommand;
};

BitbangSPI::BitbangSPI(){
#ifndef NO_SPI 
	fd_dev_mem = open("/dev/mem", O_RDWR|O_SYNC);
		if(fd_dev_mem == -1) { 
			printf("failed to open /dev/mem\n");
		}
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
#endif
}

BitbangSPI::~BitbangSPI(){
	//dio_out = 0;  // set to off -- do this different
	printf("Bitbang spi destructor\n");
#ifndef NO_SPI
	close(fd_dev_mem);
#endif
}

int BitbangSPI::spiSend(uint_8 * data, int size) {
	
	BitbangSPI::lastcommand = data[1];

#ifndef NO_SPI

	//*dio_out = 0;//make sure all off?
	dioblock_out->spi_ss = 0;//pull low to make compass listen
	dioblock_out->spi_clock = 0;//clock low
	PAUSE;
	for(int byte = 0;byte < size; byte++){
		for (int i=0;i<8;i++){

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
#endif
	#ifdef PRINT_TO_SCREEN_Tx
		printf("sent:\n");
		for(int i=0; i<size;i++){
			printf("\tpacket %i = %X\n",i,data[i]);
		}
	#endif
	return(0);
}


int BitbangSPI::spiGet(uint_8 * dataresp, int size){
#ifndef NO_SPI
	uint_8 data = 0;
	dioblock_out->spi_ss = 0;//pull low to make compass send?
	dioblock_out->spi_clock = 0;//clock low
	dioblock_out->spi_mosi = 0;//supposed to send 0x00 when revieving
	PAUSE;

	for(int byte = 0;byte < size; byte++){
		for(int i=0;i<8;i++){
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
#endif

#ifdef FAKE_RESP
	s32_u8 temps32;
	char_u8 tempchr;
	if (BitbangSPI::lastcommand == get_mod_info){
		//uint_8 dataresp[11];
		dataresp[0] = sync_flag;
		dataresp[1] = mod_info_resp;
		tempchr.chr = 'V';
		dataresp[2] = tempchr.u8;
		tempchr.chr = '2';
		dataresp[3] = tempchr.u8;
		tempchr.chr = 'X';
		dataresp[4] = tempchr.u8;
		tempchr.chr = 'e';
		dataresp[5] = tempchr.u8;
		tempchr.chr = 'V';
		dataresp[6] = tempchr.u8;
		tempchr.chr = '2';
		dataresp[7] = tempchr.u8;
		tempchr.chr = '0';
		dataresp[8] = tempchr.u8;
		tempchr.chr = '1';
		dataresp[9] = tempchr.u8;
		dataresp[10] = terminator;
	}
	
	if (BitbangSPI::lastcommand == get_data){
		dataresp[0] = sync_flag;
		dataresp[1] = data_resp;
		dataresp[2] = 2;//compcount
       
		dataresp[3] = XRaw;
		temps32.s32 = 500;
		       dataresp[4] = temps32.u8[0];
		       dataresp[5] = temps32.u8[1];
		       dataresp[6] = temps32.u8[2];
		       dataresp[7] = temps32.u8[3];
	
		dataresp[8] = YRaw;
		temps32.s32 = 500;
		       dataresp[9] = temps32.u8[0];
		       dataresp[10] = temps32.u8[1];
		       dataresp[11] = temps32.u8[2];
		       dataresp[12] = temps32.u8[3];

		dataresp[13] = terminator;
	}

	if(BitbangSPI::lastcommand == get_config){
		dataresp[0] = sync_flag;
		dataresp[1] = config_resp;
		dataresp[2] = true_north;
		dataresp[3] = false;
		dataresp[4] = terminator;
		
	}

	if(BitbangSPI::lastcommand == get_cal_data){
		dataresp[0] = sync_flag;
		dataresp[1] = cal_data_resp;
		dataresp[2] = 24;
			for(int i =3;i<27;i++){
				dataresp[i] = 0;
			}
		dataresp[27] = terminator;
		
	}
#endif

#ifdef PRINT_TO_SCREEN_Rx
	printf("received:\n");
	for(int i=0; i<size;i++){
		printf("\tpacket %i = %X\n",i,dataresp[i]);
	}
#endif

	return(0);
}

#endif //BITBANG_SPI_H
