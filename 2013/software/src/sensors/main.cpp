#include "sensor.h"


int main(int argc, char **argv) {
	sensor datSensor;
	datSensor.open("/dev/ttyGPS", 4800);
}

