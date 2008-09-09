#define ATMEGA168

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/signal.h>
#include <avr/delay.h>
//#include <stdio.h>
#include <stdarg.h>

/* PIN DEFINITIONS */
/** SPI **/
#define SPI_MOSI 						/*13*/
#define SPI_MISO						7/*12*/
#define SPI_CLK							6/*11*/
#define SPI_SS_LEFT_MOTOR_ENCODER		5/*10*/
#define SPI_SS_RIGHT_MOTOR_ENCODER		4/*9*/
#define BITBANG_SPI 1

/* MOTOR ENCODER DEFINITIONS */
#define COUNTER_SCALER			(64)
#define COUNTER_RATE			(F_CPU/COUNTER_SCALER)
#define TOTAL_ENCODER_TICKS ((double)512)
#define RAD_PER_ENCODER_TICK	((double)TWO_PI/TOTAL_ENCODER_TICKS)
#define LEFT_MOTOR_ENCODER_DIRECTION 1
#define RIGHT_MOTOR_ENCODER_DIRECTION 0

/* ROBOT PHYSICAL DEFINITIONS */  /*TODO: determine these values */
#define MOTOR_RATIO		((double)20)
#define WHEEL_RADIUS		((double)5 / (double)MOTOR_RATIO)
#define WHEEL_BASE			((double)28)


//TODO:
//	- generalize this to work for all of our code
//	- - pull/push data, read/set one/all/several values, fixed/variable length messages
//	- write SPI library
//	- comment everything


//TODO: make this a typedef (why doesn't it work?)
struct motorEncoderData{
	int leftMotorTick;
	int rightMotorTick;
	unsigned int time; //this should be made an int, if possible
};

/* VARIABLES */
/* SERIAL */
int incomingByte = 0;
/* MOTOR ENCODERS */
struct motorEncoderData current = {}; // TODO: rename these
struct motorEncoderData previous = {};
double heading = 0;

void setup(void) {
	/* open the serial port */
	Serial.begin(9600);
	
	/* set the pin modes */ //TODO: do this with a loop (?)
	/* SPI */
	pinMode(SPI_MISO, INPUT);
	//pinMode(SPI_MOSI, OUTPUT);
	pinMode(SPI_SS_LEFT_MOTOR_ENCODER, OUTPUT);
	pinMode(SPI_SS_RIGHT_MOTOR_ENCODER, OUTPUT);
	pinMode(SPI_CLK, OUTPUT);

	/* De-select all SPI devices */
	digitalWrite(SPI_SS_LEFT_MOTOR_ENCODER, HIGH);
	digitalWrite(SPI_SS_LEFT_MOTOR_ENCODER, HIGH);

	// Can this be removed?
	TCCR1A = 0;//set to counter mode
	//TCCR1B = (1<<CS12)|(1<<CS10);// clk1o/1024 table on pg134
	//TCCR1B = (1<<CS10);// no prescale clk1o/1
	TCCR1B = (1<<CS11)|(1<<CS10);//clkio/64
}

void loop(void) {

	// TODO: don't do this if data is bad
	previous = current;
	readMotorEncoders(&current);

	calcHeading();
	readSerial();
}


void readMotorEncoders(struct motorEncoderData *data) {

	unsigned int rawData;

	data->time = getTime(); // WARNING: the left and right values aren't measured at exactly the same time

	rawData = SPIReadInt(SPI_MISO, SPI_SS_LEFT_MOTOR_ENCODER, SPI_CLK);
	data->leftMotorTick = convertMotorEncoderFormat(rawData);

	/*Serial.print("Left Raw: ");
	Serial.print(rawData, BIN);
	Serial.print("\t");*/

	rawData = SPIReadInt(SPI_MISO, SPI_SS_RIGHT_MOTOR_ENCODER, SPI_CLK);
	data->rightMotorTick = convertMotorEncoderFormat(rawData);
	/*Serial.print("Right Raw: ");
	Serial.print(rawData, BIN);
	Serial.print("\t\n");*/
}

void calcHeading(void) {
	// TODO: make max value smaller
	int dl = delta(current.leftMotorTick, previous.leftMotorTick, 100, TOTAL_ENCODER_TICKS - 1,  LEFT_MOTOR_ENCODER_DIRECTION); 
	int dr = delta(current.rightMotorTick, previous.rightMotorTick, 100, TOTAL_ENCODER_TICKS - 1, RIGHT_MOTOR_ENCODER_DIRECTION);

	/*unsigned int dt;
	if (t2 >= t1) {
		dt = t2 - t1;
	} else if (t2 < t1) {
		dt = ( t2 + 65536 ) - t1;
  	}*/

	/* Formula taken from http://rossum.sourceforge.net/papers/DiffSteer/
	 *		This assumes that the robot drives a cirular path, without slipping.
	 */
	heading += ( ((double)(dr - dl)) * (double)RAD_PER_ENCODER_TICK * (double)WHEEL_RADIUS / (double)WHEEL_BASE );
	while (heading >= TWO_PI) { // this could be done more effiecently
		heading -= TWO_PI;
	}

	while (heading < 0) {
		heading += TWO_PI;
	}

	//Serial.print("Left: ");
	//Serial.print(current.leftMotorTick);
	//Serial.print("\t");
	//Serial.print("Right: ");
	//Serial.print(current.rightMotorTick);
	//Serial.print("\t");
	//Serial.print("dl: ");
	//Serial.print(dl);
	//Serial.print("\t");
	//Serial.print("dr: ");
	//Serial.print(dr);
	//Serial.print("\t");
	//Serial.print("Heading: ");
	//serialPrintDouble(heading, 5); // in radians
	//Serial.print("\t");
	//serialPrintDouble(heading * 180 / (PI), 5); // in degrees
	//Serial.print("\n");
}

// must hold: maxDiff <= maxVal
int delta(int p2, int p1, int maxDiff, int maxVal, int dir) {
	int diff;
	if (dir == 0) {
		diff = p2 - p1;
	} else {
		diff = p1 - p2;
	}

	if (abs(diff) <= maxDiff) {
		return(diff);
	} else if (abs(maxVal-abs(diff)) <= maxDiff) {
		if (diff >= 0) {
			return(diff  - maxVal);
		} else {
			return(diff + maxVal);
		}
	} else {
		//error: large jump, don't use //TODO: make it not use data
		return(0);
	}
}

/* TODO: write comments
 *
 *
 *
 *
 */
int convertMotorEncoderFormat(unsigned int data) {
	//XX: trash data does not match the spec
	if( (data & 0x0078) | ~(data | 0x3F7F) ) {
		Serial.print("Frame error\n");
		//error
		return(0);
	} else {
		return( (data & 0x0007) | ((data & 0x3F00) >> 5) );
	}
}

void readSerial(void) {
	if (Serial.available() > 0) {
		incomingByte = Serial.read();
		if (incomingByte == 'r') {
			sendStatus();
		} else if (incomingByte == 'w') {
			while (Serial.available()<2){}  // TODO: add timeout
			int variableNumber = Serial.read();
			int variableValue = Serial.read();
			setVariable(variableNumber, variableValue);
		} else if (incomingByte == 'i') {
			Serial.print("e");
		} else {
			//error
		}
	}
}

//TODO: make this use an array
void setVariable(int num, int val) {
	switch (num) {
		case 0:
			heading=val;
			break;
		default:
			//error
			break;
	}
	Serial.println(num);  //TODO: check for errors
}

void sendStatus() {
	//serialPrintBytes(&(current.leftMotorTick), sizeof(int));  // only 9 bits are used
	//serialPrintBytes(&(current.rightMotorTick), sizeof(int));
	serialPrintBytes(&heading, sizeof(double));
	//serialPrintBytes(&(current.time), sizeof(int));
}

void serialPrintBytes(void *data, int numBytes) {
	for (int i = 0; i < numBytes; i++) {
		Serial.print(((unsigned char *)data)[i], BYTE);
	}
}

// Fixed point only. Should make scientific notation option.
void serialPrintDouble(double data, int precision) {
	Serial.print((int)data);
	Serial.print(".");
	for(int i = 1; i < (precision+1); i++) {
		data = (data - (double)((int)data)) * 10;
		Serial.print((int)data);
	}
}

/* Get a single integer. Not multi-thread safe.*/
//TODO: change this to SPIReadBytes
int SPIReadInt(int inputPin, int slaveSelectPin, int clockPin) {
//#ifdef BITBANG_SPI
  	unsigned int data = 0;

	digitalWrite(slaveSelectPin, LOW);
	delayMicroseconds(500);//delay 100us before starting clock

	for(int i = 0; i < 16; i++){
		digitalWrite(clockPin, HIGH);
		delayMicroseconds(10);//delay of 10us before data is sent

		//int bit = digitalRead(inputPin);
		//Serial.print(bit);
		data |= digitalRead(inputPin) << (15 - i);
		//data |= bit << (15 - i);


		digitalWrite(clockPin, LOW);
		delayMicroseconds(50);//wait to make this a 20khz signal (encoder goes to 50khz)
	}
	digitalWrite(slaveSelectPin, HIGH);

	delayMicroseconds(50);//the last bit is held for 50us
	delayMicroseconds(1000);//data only refreshed every 1 ms -- note: delay(1) is inaccurate for a 1 ms wait, seems to be only accurate to about max 200us

	//Serial.print("\t");

	return(data);
//#else
//#ifdef HARD_SPI
	
//#else
//	#error "SPIReadInt: SPI mode not set.\n"
//#endif
//#endif
}

unsigned int getTime(){
	unsigned int time;
	//TCNT1 is a 16 bit timer/counter ~ pg 121
	time = TCNT1L;//how often does this tick? tests say clkio = F_CPU.
	time |= TCNT1H << 8;
	return(time);
}

void resetTime(){
	TCNT1H = 0;
	TCNT1L = 0;
}

