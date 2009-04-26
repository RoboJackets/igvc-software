#include "SonarInterface.h"
#include <stdlib.h>

SonarInterface sdev;

int main(void){


	int data[10];
	int dat;	
	printf("\nInitializing Link...\n");


	sdev.initSonDev(11,12,2000,14);

	printf("Pinging all sonars, then waiting for data return.\nShould return 11 rvalues (0-10).\n");
	sdev.pingAll();
	while(sdev.updateAll() == -1);
	for(int i=0; i<11; i++){
		printf("%d:%d\n",i,sdev.readIndiv(i,'o'));
	}
	printf("NEXT>\n");
	getchar();

	printf("Deactivating every other sonar and repeating same process.\nShould return rvalues for 0,2,4,6,8,10.\n");
	sdev.setIndivActive(1,0); sdev.setIndivActive(3,0); sdev.setIndivActive(5,0); sdev.setIndivActive(7,0); sdev.setIndivActive(9,0);
	sdev.pingAll();
	while(sdev.updateAll() == -1);
	for(int i=0; i<11; i++){
		printf("%d:%d\n",i,sdev.readIndiv(i,'o'));
	}
	printf("NEXT>\n");
	getchar();

	printf("Switching Actives with deactives, and vice versa.\nShould return rvalues for 1,3,5,7,9.\n");
	sdev.setIndivActive(1,1); sdev.setIndivActive(3,1); sdev.setIndivActive(5,1); sdev.setIndivActive(7,1); sdev.setIndivActive(9,1);
	sdev.setIndivActive(0,0); sdev.setIndivActive(2,0); sdev.setIndivActive(4,0); sdev.setIndivActive(6,0); sdev.setIndivActive(8,0); sdev.setIndivActive(10,0);
	sdev.pingAll();
	while(sdev.updateAll() == -1);
	for(int i=0; i<11; i++){
		printf("%d:%d\n",i,sdev.readIndiv(i,'o'));
	}
	printf("NEXT>\n");
	getchar();

	printf("Reading gain from sonar 0 and 8. Should return 12 for both\n");
	dat = sdev.getIndivGain(0);
	printf("g0:%d\n",dat);
	dat = sdev.getIndivGain(8);
	printf("g8:%d\n",dat);
	printf("NEXT>\n");
	getchar();

	printf("Setting all gain to 3, then changing 5's gain to 8. Reading 0,5,8's gain. Should return 3,8,3\n");
	sdev.setIndivActive(0,1); sdev.setIndivActive(2,1); sdev.setIndivActive(4,1); sdev.setIndivActive(6,1); sdev.setIndivActive(8,1); sdev.setIndivActive(10,1);
	sdev.setAllGain(3);
	sdev.setIndivGain(5,8);
	dat = sdev.getIndivGain(0);
	printf("g0:%d\n",dat);
	dat = sdev.getIndivGain(5);
	printf("g5:%d\n",dat);
	dat = sdev.getIndivGain(8);
	printf("g8:%d\n",dat);
	printf("NEXT>\n");
	getchar();

	printf("Setting all mrange to 1000, then changing 6's mrange to 500. Reading 1,7,6's mrange. Should return approx 1K,1K,500\n");
	sdev.setAllMRange(1000);
	sdev.setIndivMRange(6,500);
	dat = sdev.getIndivMRange(1);
	printf("mr1:%d\n",dat);
	dat = sdev.getIndivMRange(7);
	printf("mr7:%d\n",dat);
	dat = sdev.getIndivMRange(6);
	printf("mr6:%d\n",dat);
	printf("NEXT>\n");
	getchar();

	printf("Setting autopilot on for 3 iterations\n");
	sdev.setAutopilot(1);
	while(sdev.updateAll() == -1);
	for(int i=0; i<11; i++){
		printf("%d:%d\n",i,sdev.readIndiv(i,'o'));
	}
	while(sdev.updateAll() == -1);
	for(int i=0; i<11; i++){
		printf("%d:%d\n",i,sdev.readIndiv(i,'o'));
	}
	while(sdev.updateAll() == -1);
	for(int i=0; i<11; i++){
		printf("%d:%d\n",i,sdev.readIndiv(i,'o'));
	}
	sdev.setAutopilot(0);
	printf("Next three should return exact same values. Autopilot has been turned off.\n");
	while(sdev.updateAll() == -1);
	for(int i=0; i<11; i++){
		printf("%d:%d\n",i,sdev.readIndiv(i,'o'));
	}
	while(sdev.updateAll() == -1);
	for(int i=0; i<11; i++){
		printf("%d:%d\n",i,sdev.readIndiv(i,'o'));
	}
	while(sdev.updateAll() == -1);
	for(int i=0; i<11; i++){
		printf("%d:%d\n",i,sdev.readIndiv(i,'o'));
	}
	printf("NEXT>\n");
	getchar();

	printf("Lowering frequency to 2. Setting Top and bottom half to respective steps. Turning autopilot on for 6 iterations\n");
	printf("NEXT>\n");
	getchar();
	sdev.setFreq(2);
	sdev.setStepTotal(2);
	sdev.setIndivStep(0,0);
	sdev.setIndivStep(1,1);
	sdev.setIndivStep(2,0);
	sdev.setIndivStep(3,1);
	sdev.setIndivStep(4,0);
	sdev.setIndivStep(5,1);
	sdev.setIndivStep(6,0);
	sdev.setIndivStep(7,1);
	sdev.setIndivStep(8,0);
	sdev.setIndivStep(9,1);
	sdev.setIndivStep(10,0);
	sdev.setAutopilot(1);
	while(sdev.updateAll() == -1);
	for(int i=0; i<11; i++){
		printf("%d:%d\n",i,sdev.readIndiv(i,'o'));
	}
	while(sdev.updateAll() == -1);
	for(int i=0; i<11; i++){
		printf("%d:%d\n",i,sdev.readIndiv(i,'o'));
	}
	while(sdev.updateAll() == -1);
	for(int i=0; i<11; i++){
		printf("%d:%d\n",i,sdev.readIndiv(i,'o'));
	}
	while(sdev.updateAll() == -1);
	for(int i=0; i<11; i++){
		printf("%d:%d\n",i,sdev.readIndiv(i,'o'));
	}
	while(sdev.updateAll() == -1);
	for(int i=0; i<11; i++){
		printf("%d:%d\n",i,sdev.readIndiv(i,'o'));
	}
	while(sdev.updateAll() == -1);
	for(int i=0; i<11; i++){
		printf("%d:%d\n",i,sdev.readIndiv(i,'o'));
	}
	sdev.setAutopilot(0);
	printf("NEXT>\n");
	getchar();

	printf("getting map of system\n");
	for(int i = 0;i<11;i++){
		dat = sdev.getIndivMRange(i);
		printf("mr%d:%d\n",i,dat);
		dat = sdev.getIndivGain(i);
		printf("g%d:%d\n",i,dat);
		dat = sdev.getIndivActive(i);
		printf("a%d:%d\n",i,dat);
		dat = sdev.getIndivStep(i);
		printf("st%d:%d\n",i,dat);
	}
	dat = sdev.getFreq();	
	printf("fr:%d\n",dat);
	dat = sdev.getStepTotal();
	printf("stot:%d\n",dat);
	printf("NEXT>\n");
	getchar();

	printf("Success!\nProgram Terminated.\n");
	return 0;
}
