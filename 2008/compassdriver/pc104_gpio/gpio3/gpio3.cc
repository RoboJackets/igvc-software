#include "gpio3.hh"
#include <unistd.h>

int main(void){
	GPIO gpiodriver;
	
	gpiodriver.gpio_a_direction->uint32 = 0;
	gpiodriver.gpio_a_data->uint32 = 0;
	gpiodriver.gpio_a_mux->uint32 = 0;

	gpiodriver.gpio_a_direction->pin0 = 1;//assuming 1 = out
	gpiodriver.gpio_a_direction->pin30 = 1;

	gpiodriver.gpio_a_data->pin0 = 1;//assuming 1 = on
	gpiodriver.gpio_a_data->pin30 = 1;

	while(1){
		gpiodriver.gpio_a_data->pin0 = 1;
		usleep(1000);
		gpiodriver.gpio_a_data->pin0 = 0;
		usleep(1000);
	}
}
