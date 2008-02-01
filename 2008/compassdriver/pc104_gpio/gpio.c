#include<unistd.h>
#include<sys/types.h>
#include<sys/mman.h>
#include<stdio.h>
#include<fcntl.h>
#include<string.h>
#include<unistd.h>

/*
using peekpoke and this program, i can set mux and direction for all four registers.  data will not write for any of A-D in peekpoke or here.


*/

//set A is on the 64-pin header, closest to the 40 pin header.  pin 1 is closest to serial port, pin 32 other side down the row (linearly)
#define PC104_A_DATA 0x10/sizeof(int) //data register
#define PC104_A_DIRECTION 0x20/sizeof(int) //direction register, 1 - output
#define PC104_A_MUX 0x30/sizeof(int) //function register; 0 = gpio, 1 = ISA, 2,3 = reserved.  pins are configured in sets of two (bits 0&1 configure pins 1&2)

#define SYSCONN_BASE 0xe8000000 //system controller base addr

int main(void){
	volatile unsigned int *gpio_a_data, *gpio_a_direction, *gpio_a_mux;
	//unsigned int *start;
	int fd = open("/dev/mem", O_RDWR|O_SYNC);

	unsigned int * start = (unsigned int *)mmap(0, getpagesize(), PROT_READ|PROT_WRITE, MAP_SHARED, fd, SYSCONN_BASE);

/*
	gpio_a_data =  (unsigned int *)(start + PC104_A_DATA);  // dio 1
	gpio_a_direction = (unsigned int *)(start + PC104_A_DIRECTION);
	gpio_a_mux = (unsigned int *)(start + PC104_A_MUX);
*/

	gpio_a_data =  &start[PC104_A_DATA];  // dio 1
	gpio_a_direction = &start[PC104_A_DIRECTION];
	gpio_a_mux = &start[PC104_A_MUX];


	printf("gpio_a_data before init: %X\n", *gpio_a_data);
	printf("gpio_a_direction before init: %X\n", *gpio_a_direction);
	printf("gpio_a_mux before init: %X\n", *gpio_a_mux);

	*gpio_a_data = 0x0;
	*gpio_a_direction = 0x7FFFFFFF;//which way is output?
	*gpio_a_mux = 0;//set pc104_A pins 1-32 to gpio

	printf("gpio_a_data after init: %X\n", *gpio_a_data);
	printf("gpio_a_direction after init: %X\n", *gpio_a_direction);
	printf("gpio_a_mux after init: %X\n", *gpio_a_mux);


while(1){
	*gpio_a_data = 0xFFFFFFFF;
	//*gpio_a_data |= 1 << 0;
	printf("gpio_a_data after on:\t%X\n", *gpio_a_data);
	printf("gpio_a_direction after on:\t%X\n", *gpio_a_direction);
	printf("gpio_a_mux after on:\t%X\n", *gpio_a_mux);
	usleep(1000000);

	*gpio_a_data = 0;
	//*gpio_a_data &= ~(1 << 0);
	printf("gpio_a_data after 0:\t%X\n", *gpio_a_data);
	printf("gpio_a_direction after 0:\t%X\n", *gpio_a_direction);
	printf("gpio_a_mux after 0:\t %X\n", *gpio_a_mux);
	usleep(1000000);
	printf("\n\n");
}

	close(fd);
	return(0);
}

