#include "dio.hh"

#include<unistd.h>
#include<sys/types.h>
#include<sys/mman.h>
#include<stdio.h>
#include<fcntl.h>
#include<string.h>
#include<unistd.h>

//  * base + 0x40: free-running microseconds counter (RO)//could be useful for a busy loop

#define WAIT usleep(1000)

int main(void){
	DIO_basic diodriver;

/*	while(true){
		diodriver.dioblock_out->greenled = 1;
		WAIT;
		diodriver.dioblock_out->greenled = 0;
		WAIT;
	}
*/

//	for(int i = 0; i < 1000; i++){

	//diodriver.dioblock_in->pin13 = 1;//set as output
	while(1){
		diodriver.dioblock_out->pin13 = 1;
		//diodriver.dioblock_out->greenled = 1;
		WAIT;
		diodriver.dioblock_out->pin13 = 0;
		WAIT;
	}

	return(0);
}
