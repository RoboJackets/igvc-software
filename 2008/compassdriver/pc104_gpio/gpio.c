#include<unistd.h>
#include<sys/types.h>
#include<sys/mman.h>
#include<stdio.h>
#include<fcntl.h>
#include<string.h>
#include<unistd.h>


//set A is on the 64-pin header, closest to the 40 pin header.  pin 1 is closest to serial port, pin 32 other side down the row (linearly)
#define PC104_A_DATA 0x10 //data register
#define PC104_A_DIRECTION 0x20 //direction register
#define PC104_A_MUX 0x30 //function register; 0 = gpio, 1 = ISA, 2,3 = reserved.  pins are configured in sets of two (bits 0&1 configure pins 1&2)

#define SYSCONN_BASE 0xe8000000 //system controller base addr

int main(void){
	volatile unsigned int *gpio_a_data, *gpio_a_direction, *gpio_a_mux;
	unsigned char *start;
	int fd = open("/dev/mem", O_RDWR|O_SYNC);

	start = mmap(0, getpagesize(), PROT_READ|PROT_WRITE, MAP_SHARED, fd, SYSCONN_BASE);

        gpio_a_data =  (unsigned int *)(start + PC104_A_DATA);  // dio 1
        gpio_a_direction = (unsigned int *)(start + PC104_A_DIRECTION);
	gpio_a_mux = (unsigned int *)(start + PC104_A_MUX);

        *gpio_a_data = 0;
	*gpio_a_direction = 1;//which way is output?
	*gpio_a_mux = 0;//set pc104_A pins 1-32 to gpio


while(1){
        //*dio |= 0xFFFFFFFF;
        *gpio_a_data |= 1 << 0;
        usleep(1000);
        *gpio_a_data &= ~(1 << 0);
        usleep(1000);
}

	close(fd);
	return(0);
}

