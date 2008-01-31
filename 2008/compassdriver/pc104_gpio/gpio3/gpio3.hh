#ifndef GPIO3_HH
#define GPIO3_HH

#include<unistd.h>
#include<sys/types.h>
#include<sys/mman.h>
#include<cstdio>
#include <cstdlib>
#include<fcntl.h>
#include<string.h>



//set A is on the 64-pin header, closest to the 40 pin header.  pin 1 is closest to serial port, pin 32 other side down the row (linearly)
#define PC104_A_DATA 0x10 //data register offset
#define PC104_A_DIRECTION 0x20 //direction register offset
#define PC104_A_MUX 0x30 //function register offset; 0 = gpio, 1 = ISA, 2,3 = reserved.  pins are configured in sets of two (bits 0&1 configure pins 1&2)

#define SYSCONN_BASE 0xe8000000 //system controller base addr

union gpio_a_block {
	struct {
		unsigned pin0:1;
		unsigned pin1:1;
		unsigned pin2:1;
		unsigned pin3:1;
		unsigned pin4:1;
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
		unsigned pin17:1;
		unsigned pin18:1;
		unsigned pin19:1;
		unsigned pin20:1;
		unsigned pin21:1;
		unsigned pin22:1;
		unsigned pin23:1;
		unsigned pin24:1;
		unsigned pin25:1;
		unsigned pin26:1;
		unsigned pin27:1;
		unsigned pin28:1;
		unsigned pin29:1;
		unsigned pin30:1;
		unsigned :1;//ground
	};
	unsigned uint32;
};

union gpio_a_mux_block{
	struct {
		unsigned pin0_1:2;
		unsigned pin2_3:2;
		unsigned pin4_5:2;
		unsigned pin6_7:2;
		unsigned pin8_9:2;
		unsigned pin10_11:2;
		unsigned pin12_13:2;
		unsigned pin14_15:2;
		unsigned pin16_17:2;
		unsigned pin18_19:2;
		unsigned pin20_21:2;
		unsigned pin22_23:2;
		unsigned pin24_25:2;
		unsigned pin26_27:2;
		unsigned pin28_29:2;
		unsigned pin30_31:2;
	};
	unsigned int uint32;
};

class GPIO {
	public:
		volatile gpio_a_block * gpio_a_data;
		volatile gpio_a_block * gpio_a_direction;
		volatile gpio_a_mux_block * gpio_a_mux;
		
		GPIO(void);
		//~GPIO(void);

};

GPIO::GPIO(void){
	unsigned char *start;
	int fd = open("/dev/mem", O_RDWR|O_SYNC);	

	start = (unsigned char *)mmap(0, getpagesize(), PROT_READ|PROT_WRITE, MAP_SHARED, fd, SYSCONN_BASE);

	gpio_a_data =  (volatile gpio_a_block *)(start + PC104_A_DATA);
	gpio_a_direction = (volatile gpio_a_block *)(start + PC104_A_DIRECTION);
	gpio_a_mux = (volatile gpio_a_mux_block *)(start + PC104_A_MUX);


	gpio_a_data->uint32 = 0;
	gpio_a_direction->uint32 = 0;//which way is output?
	gpio_a_mux->uint32 = 0;//set pc104_A pins 1-32 to gpio
	

	close(fd);
}
/*
GPIO::~GPIO(void){

}
*/


#endif //GPIO3_HH
