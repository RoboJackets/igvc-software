#include "WProgram.h"
#define ATMEGA168


//after build -- to flash
//avr-objcopy -O ihex -R .eeprom blink.out blink.hex
//sudo avrdude -V -c dragon_isp -p m168 -b 19200 -P usb -U flash:w:cmaketest.hex

//http://zedcode.blogspot.com/2007/02/gcc-c-link-problems-on-small-embedded.html
//to keep from needing to link to c++ std lib, maybe? not totally sure
// solves this error:
//../../libarduino/libarduinocore.a(Print.o):(.data+0x6): undefined reference to `__cxa_pure_virtual'
extern "C" void __cxa_pure_virtual(void)
{
	// call to a pure virtual function happened ... wow, should never happen ... stop
	for(;;)
	{

	}
}

int main()
{
	init();

	Serial.begin(9600);
	pinMode(13, OUTPUT);
	int pin = HIGH; 
	for(;;)
	{
		Serial.println("Hello");
		digitalWrite(13, pin);
		(pin == HIGH) ? (pin = LOW) : (pin = HIGH);
		delay(1000);
	}
	return 0;
}
