#include "SonarInterface.h"
#include <stdlib.h>

SonarInterface sdev;

int main(void){

	int data[10];
	int dat;	
	printf("\nInitializing Link...");
	sdev.setFreq(14);

	sdev.setStepTotal(4);
	sdev.setIndivStep(0,0); sdev.setIndivActive(0,0);
	sdev.setIndivStep(1,0); sdev.setIndivActive(1,1);
	sdev.setIndivStep(2,0); sdev.setIndivActive(2,1);
	sdev.setIndivStep(3,1); sdev.setIndivActive(3,1);
	sdev.setIndivStep(4,1); sdev.setIndivActive(4,1);
	sdev.setIndivStep(5,2); sdev.setIndivActive(5,1);
	sdev.setIndivStep(6,2); sdev.setIndivActive(6,1);
	sdev.setIndivStep(7,2); sdev.setIndivActive(7,1);
	sdev.setIndivStep(8,3); sdev.setIndivActive(8,1);
	sdev.setIndivStep(9,3); sdev.setIndivActive(9,1);

	sdev.pingindiv(0);
	

	sdev.setIndivGain(0,15);
	sdev.pingall();
	for(int i=0; i<10; i++){
		dat = sdev.readindiv(i);
		printf(":s:>%d--%d\n",i,dat);
	}
	dat = sdev.getIndivGain(0);
	printf(":s:>%dg--%d\n",0,dat);
	sdev.setAllGain(30);
	sdev.setIndivMRange(0,1500);
	sdev.pingall();
	for(int i=0; i<10; i++){
		dat = sdev.readindiv(i);
		printf(":s:>%d--%d\n",i,dat);
	}
	dat = sdev.getIndivGain(0);
	printf(":s:>%dg--%d\n",0,dat);
	dat = sdev.getIndivMRange(0);
	printf(":mr:>%dm--%d\n",0,dat);
	dat = sdev.getIndivActive(0);
	printf(":a0:>%d\n",dat);
	dat = sdev.getIndivActive(1);
	printf(":a1:>%d\n",dat);
	dat = sdev.getIndivStep(0);
	printf(":s0:>%d\n",dat);
	dat = sdev.getIndivStep(5);
	printf(":s1:>%d\n",dat);
	dat = sdev.getFreq();
	printf(":f:>%d\n",dat);
	dat = sdev.getStepTotal();
	printf(":st:>%d\n",dat);
	
	printf("Success!\nProgram Terminated.\n");
	return 0;
}
