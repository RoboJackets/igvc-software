#include "getadc.h"

int main(void){
	GetADC adc;
	int channels[3] = {0,1,2};
	adc.getdata(channels, 3);

	for(int i = 0; i < 1024; i++) {
		printf("raw: %x\tconverted: %x\n",adc.sramout[i], adc.sram_volt_out[i]);//converted
	}
	//adc.sramout//the data printed to screen
	//adc.sram_volt_out//the data printed to screen converted to voltage
	return(0);
}
