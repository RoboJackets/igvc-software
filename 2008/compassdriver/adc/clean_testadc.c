#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <assert.h> 
#include <string.h>

//main and twifunc depend on these
	volatile unsigned int *data, *control, *status;
	unsigned int verbose = 0, addr;
//

//EWD: Clean up this header file...
#include "twsi.h"
#include "twifunc.h"

int setchan(int * chan, int length){
	int chanout = 0;	
	int i = 0;
	for(i = 0; i < length;i++){
		chanout |= 1<<(chan[i]);
	}
	//channel 6 => bit 4
	if(chanout & (1<<6)) {
		chanout |= 1<<4;
		chanout &= ~(1<<6);
	}
	//channel 7 => bit 5
	if(chanout & (1<<7)) {
		chanout |= 1<<5;
		chanout &= ~(1<<7);
	}
	return(chanout);
}


getadc(int chanin){
	volatile unsigned int *fpga, *sram;
	unsigned int i;
	int chans;
	
	fpga = (unsigned int *)mmap(0, 0x1004, PROT_READ|PROT_WRITE,
	  MAP_SHARED, devmem, 0xE8100000);
       	sram = &fpga[SRAM];

	chans = setchan(chanin, 4);
	if(chans < 0) return -1;

	twi_select(AVR_ADDR, WRITE);
	twi_write(chans | 0x40);
	*control = STOP|TWSIEN; // Send stop signal

	if(verbose) fprintf(stderr,"Sent Stop signal\n");

//EWD: Implement this in a more elegant way. That is write out all FF's then print values as they are available.
//2KSPS => a new sample is available every 500uS
	usleep(1500000);
	for(i=0;i<1024;i++) {
		if(raw) {
			printf("0x%x\n", sram[i]);
		}
	}

	for(i=0;i<1024;i++) {
		if(raw) {
			printf("converted: %u\n", (sram[i]*1.1)/1024);
		}
	}
	//return(sram);

}


int main(int argc, char **argv) {
	
	int devmem;
	unsigned int *twi_regs, *regs;
	unsigned int start_adc=0, raw=0;

	devmem = open("/dev/mem", O_RDWR|O_SYNC);
	assert(devmem != -1);
	twi_regs = (unsigned int *)mmap(0, getpagesize(), PROT_READ|PROT_WRITE,
	  MAP_SHARED, devmem, 0xF1011000);

	regs = (unsigned int *)mmap(0, getpagesize(), PROT_READ|PROT_WRITE,
	  MAP_SHARED, devmem,  0xe8000000);


        control = &twi_regs[CONTROL];
        data = &twi_regs[DATA];
        status = &twi_regs[STATUS];
	

	int channels[4] = {0,1,2,3};
	getadc(channels);

	return 0;
}
