#include<unistd.h>
#include<sys/types.h>
#include<sys/mman.h>
#include<stdio.h>
#include<fcntl.h>
#include<string.h>
#include<unistd.h>

#define PC104A_IN_OUT 0x10
#define PC104A_DIRECTION 0x20
#define PC104A_MUX 0x30

int main(int argc, char **argv)
{
   volatile unsigned int *gpio_a_data, *gpio_a_direction, *gpio_a_mux;
   unsigned char *start;
   int fd = open("/dev/mem", O_RDWR|O_SYNC);

   start = mmap(0, getpagesize(), PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0xe8000000);
        gpio_a_data =  (unsigned int *)(start + PC104A_IN_OUT);  // dio 1
        gpio_a_direction = (unsigned int *)(start + PC104A_DIRECTION);
	gpio_a_mux = (unsigned int *)(start + PC104A_MUX);

        *gpio_a_data = 0;
	*gpio_a_direction = 1;
	*gpio_a_mux = 0;//set isa A pins 1-31 to gpio
	

while(1){
        //*dio |= 0xFFFFFFFF;
        *gpio_a_data |= 1 << 0;
        usleep(1000);
        *gpio_a_data = 0; // &  ~(1 << 16);
        usleep(1000);
}

   close(fd);
   return 0;
}

