#include "workingdio.hh"
#include<unistd.h>
int main(void){
	BitbangSPI diodriver;

while(1){
	printf("%X\n",diodriver.dioblock_in->pin10);
	usleep(100000);
}


/*
while(1){
	diodriver.dioblock_out->bytes |= spi_ss_on;
	diodriver.dioblock_out->bytes |= spi_frame_on;
	diodriver.dioblock_out->bytes |= spi_miso_on;
	diodriver.dioblock_out->bytes |= spi_mosi_on;
	diodriver.dioblock_out->bytes |= spi_clock_on;
usleep(1000);
//	diodriver.dioblock_out->bytes &= spi_ss_off;
//	diodriver.dioblock_out->bytes &= spi_frame_off;
//	diodriver.dioblock_out->bytes &= spi_miso_off;
//	diodriver.dioblock_out->bytes &= spi_mosi_off;
//	diodriver.dioblock_out->bytes &= spi_clock_off;
usleep(1000);
}
*/
/*
while(1){
        diodriver.dioblock_out->spi_ss = 1;
        diodriver.dioblock_out->bytes |= spi_frame_on;
        diodriver.dioblock_out->bytes |= spi_miso_on;
        diodriver.dioblock_out->bytes |= spi_mosi_on;
        diodriver.dioblock_out->bytes |= spi_clock_on;
usleep(1000);
      diodriver.dioblock_out->spi_ss = 0;
//      diodriver.dioblock_out->bytes &= spi_frame_off;
//      diodriver.dioblock_out->bytes &= spi_miso_off;
//      diodriver.dioblock_out->bytes &= spi_mosi_off;
//      diodriver.dioblock_out->bytes &= spi_clock_off;
usleep(1000);
}
*/


	return(0);
}
