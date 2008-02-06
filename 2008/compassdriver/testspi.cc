#include <cstdio>

#include "BitbangSPI_magic.h"
#include "compassTypes.h"

int main(void){
	BitbangSPI spidriver;
	uint_8 data_out[3];
	uint_8 data_in[11];

	data_out[0] = sync_flag;
	data_out[1] = get_mod_info;
	data_out[2] = terminator;

	spidriver.spiSend(data_out, 3);
	spidriver.spiGet(data_in, 11);

	
	char_u8 tempchr;
	ModInfoResp moduleversion;

		for(int i = 2;i<6;i++) {
			tempchr.u8 = data_in[i];
			moduleversion.module_type[i-2] = tempchr.chr;
		}
		moduleversion.module_type[4] = '\0';//probably usefull
		
		for(int i = 6;i<10;i++) {
			tempchr.u8 = data_in[i];
			moduleversion.firmware_version[i-6] = tempchr.chr;
		}	       
		moduleversion.firmware_version[4] = '\0';

	printf("module type:\t%s\n", moduleversion.module_type);
	printf("firmware version:\t%s\n",moduleversion.firmware_version);

	

}
