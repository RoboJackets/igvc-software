#ifndef GET_ADC_H
#define GET_ADC_H

//derived from ts7800ctl.c
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <assert.h> 
#include <string.h>

#include "twsi.h"


#define AVR_VREF 1.1 //reference voltege for the ADC on the ATMEGA48

class GetADC{

	private:
		//twi funcs -- these were all marked static, but c++ didn't like that.	
		void inline twi_write(unsigned char dat);
		unsigned char inline twi_read();
		void inline twi_select(unsigned char addr, unsigned char dir);
		
		//make TS channel format
		int setchan(int * chan, int length);
		
		//various values both the adc code and the twi code calls
		volatile unsigned int *data, *control, *status;
		unsigned int verbose, addr;
		int devmem;
		unsigned int *twi_regs, *regs;
		
	public:
		GetADC();
		~GetADC();

		int getdata(int * chanin, int length);

		//avoid malloc
		unsigned int sramout[1024];
		unsigned float sram_volt_out[1024];
};

GetADC::GetADC(){
	devmem = open("/dev/mem", O_RDWR|O_SYNC);
	assert(devmem != -1);
	twi_regs = (unsigned int *)mmap(0, getpagesize(), PROT_READ|PROT_WRITE,
	  MAP_SHARED, devmem, 0xF1011000);

	regs = (unsigned int *)mmap(0, getpagesize(), PROT_READ|PROT_WRITE,
	  MAP_SHARED, devmem,  0xe8000000);


        control = &twi_regs[CONTROL];
        data = &twi_regs[DATA];
        status = &twi_regs[STATUS];
}

GetADC::~GetADC(){
//nothing to do...?\
//can i turn off the adc? reset the input channels?
//close /dev/mem? (ts doesn't)
}

int GetADC::setchan(int * chan, int length){
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


int GetADC::getdata(int * chanin, int length){
	volatile unsigned int *fpga, *sram;
	unsigned int i;
	int chans;
	
	fpga = (unsigned int *)mmap(0, 0x1004, PROT_READ|PROT_WRITE,
	  MAP_SHARED, devmem, 0xE8100000);
       	sram = &fpga[SRAM];

	chans = setchan(chanin, length);
	if(chans < 0) return -1;

	twi_select(AVR_ADDR, WRITE);
	twi_write(chans | 0x40);
	*control = STOP|TWSIEN; // Send stop signal

	if(verbose) fprintf(stderr,"Sent Stop signal\n");

//EWD: Implement this in a more elegant way. That is write out all FF's then print values as they are available.
//2KSPS => a new sample is available every 500uS
	usleep(1500000);
	for(i=0;i<1024;i++) {
		printf("0x%x\n", sram[i]);//raw
	}

	for(i=0;i<1024;i++) {
		printf("converted: %x\n", (sram[i] * AVR_VREF)/1024);//converted
	}

	for(i=0;i<1024;i++) {
		sramout[i] = sram[i];
		sram_volt_out[i] = ((AVR_VREF * sram[i])/1024);
	}

	return(0);
	//return(sramout);//this is a public memeber now
}

void inline GetADC::twi_write(unsigned char dat) {
	if(verbose) fprintf(stderr,"Writing data (0x%x)\n", dat);
	*data = dat;
	usleep(100);

	*control = (*control & ~IFLG) | ACK;
	usleep(100);
	while((*control & IFLG) == 0x0);        // Wait for an I2C event


	if((*status != ST_DATA_ACK) && (*status != MT_DATA_ACK )) {
		if(verbose) fprintf(stderr,"Slave didn't ACK data\n");
		exit(1);
	}

	if(verbose) fprintf(stderr,"Slave ACKed data\n");
}
	
unsigned char inline GetADC::twi_read() {


	if(verbose) fprintf(stderr, "Waiting for data from master\n");
	*control = (*control & ~IFLG) | ACK;
	while((*control & IFLG) == 0x0);	// Wait for an I2C event 
	if((*status != SR_DATA_ACK) && (*status != MR_DATA_ACK)) {
		if(verbose) fprintf(stderr, "Error reading data from master(0x%x)\n", *status);
		exit(1);
	
	}

	return (unsigned char)*data;
}

void inline GetADC::twi_select(unsigned char addr, unsigned char dir) {
	unsigned int timeout = 0;
	if(verbose) fprintf(stderr,"Attempting to send start signal\n");
	*control = START|TWSIEN;        // Send start signal
	usleep(1);
	while((*control & IFLG) == 0)
		if(timeout++ > 10000) {
			if(verbose) fprintf(stderr, "Timedout\n");
			*control = STOP|TWSIEN; // Send stop signal
			if(verbose) fprintf(stderr,"Sent Stop signal\n");
			exit(-1);
		}
	if((*status != MT_START) && (*status != MT_REP_START)) {
		if(verbose) fprintf(stderr,"Couldn't send start signal(0x%x)\n",
		  *status);
		*control = STOP|TWSIEN; // Send stop signal
		if(verbose) fprintf(stderr,"Sent Stop signal\n");
		exit(-1);
	}

	if(verbose) fprintf(stderr,"Sent start signal succesfully\n"
	  "Attempting to select slave\n");
	*data = addr | dir;     // Send SLA+W/R
	usleep(1);

	*control = (*control & ~IFLG) | ACK;
	usleep(1);

	while((*control & IFLG) == 0) ;

	if((*status != MT_SLA_ACK) && (*status != MR_SLA_ACK)) {
		if(verbose) fprintf(stderr,"Slave didn't ACK select signal(0x%x)\n", *status);
		*control = STOP|TWSIEN; // Send stop signal
		if(verbose) fprintf(stderr,"Sent Stop signal\n");
		exit(-1);
		
	}
	if(verbose) fprintf(stderr,"Succesfully selected slave\n");
}

#endif //GET_ADC_H
