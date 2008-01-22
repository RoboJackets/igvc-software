#include "getadc.h"

int main(void){
	GetADC adc;
	int channels[3] = {0,1,2};
	adc.getdata(channels, 3);
	//adc.sramout//the data printed to screen
	//adc.sram_volt_out//the data printed to screen converted to voltage
	return(0);
}
