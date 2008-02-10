include "arduinospi.hh"
include "cstdlib"

int main(void){

	ARDUIO_SPI spidriver;
	short int data;

	while(1){
		print("%X", spidriver.spiGet())
	}
	

	return(0);
}
