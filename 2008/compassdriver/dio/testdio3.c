#include<unistd.h>
#include<sys/types.h>
#include<sys/mman.h>
#include<stdio.h>
#include<fcntl.h>
#include<string.h>
#include<unistd.h>

int main(int argc, char **argv)
{
   volatile unsigned int *dio_out, *dio_in;
   unsigned char *start;
   int fd = open("/dev/mem", O_RDWR|O_SYNC);

   start = mmap(0, getpagesize(), PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0xe8000000);
        dio_out =  (unsigned int *)(start + 0x8);  // dio 1
        dio_in = (unsigned int *)(start + 0x4);

        *dio_in = 0;
	
	*dio_out = 0;
	*dio_out = 1;

while(1){
        printf("dio in:%X\n",*dio_in);
	printf("dio out:%X\n\n",*dio_out);
	usleep(1000000);
}

   close(fd);
   return 0;
}

