#include "dio.hh"

#include<unistd.h>
#include<sys/types.h>
#include<sys/mman.h>
#include<stdio.h>
#include<fcntl.h>
#include<string.h>
#include<unistd.h>

//  * base + 0x40: free-running microseconds counter (RO)//could be useful for a busy loop

#define WAIT usleep(100000)

int main(void){
	DIO_basic diodriver;
		diodriver.dioblock_out->bytes = 0;
		diodriver.dioblock_out->bytes = 1 << 0;
		diodriver.dioblock_in->bytes = 1 << 10;
		while(true){
			char temp = diodriver.dioblock_in->bytes;
			temp >>= 10;
			temp = temp && 1;
			printf("%X\n", temp);
			WAIT;
		}
	return(0);
}
