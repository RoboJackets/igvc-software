#ifndef DIO_HH
#define DIO_HH

#include<unistd.h>
#include<sys/types.h>
#include<sys/mman.h>
#include<stdio.h>
#include<fcntl.h>
#include<string.h>
#include<unistd.h>


#define SYSCONT_BASE 0xe8000000
#define INPUT_OFFSET 0x4
#define OUTPUT_OFFSET 0x8

//this is defined in another header -- is that bad?
typedef unsigned char uint_8;

/*
typedef struct dio_pins{
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
	unsigned:14;
	unsigned greenled:1;
	unsigned:1;
};
*/


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


class DIO_basic{
	private:
		int fd_dev_mem;

	public:
		DIO_basic();
		~DIO_basic();
		volatile dio_pins *dioblock_in;
		volatile dio_pins *dioblock_out;

		volatile unsigned *dio_in;
		volatile unsigned *dio_out;

};

DIO_basic::DIO_basic(){
	fd_dev_mem = open("/dev/mem", O_RDWR|O_SYNC);
	unsigned int *base;
	base = (unsigned int *) mmap(0, getpagesize(), PROT_READ|PROT_WRITE, MAP_SHARED, fd_dev_mem, SYSCONT_BASE);

	dio_in = (volatile unsigned *) (base + INPUT_OFFSET);//cast as struct
	dio_out = (volatile unsigned *) (base + OUTPUT_OFFSET);

	dioblock_in = (dio_pins *) (base + INPUT_OFFSET);//cast as struct
	dioblock_out = (dio_pins *) (base + OUTPUT_OFFSET);

/*
	dioblock_in = (dio_pins *) (base + INPUT_OFFSET);//cast as struct
	dioblock_out = (dio_pins *) (base + OUTPUT_OFFSET);
*/
	//dioblock_in->bytes = 0;

}

DIO_basic::~DIO_basic(){
	//dio_out = 0;  // set to off -- do this different
	close(fd_dev_mem);

}


#endif //DIO_HH
