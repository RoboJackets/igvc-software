#define ATMEGA168

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/signal.h>
#include <avr/delay.h>
//#include <stdio.h>
#include <stdarg.h>

#include "EncoderDefines.h"
#include "EncoderFunc.h"

/* PIN DEFINITIONS */
/** SPI **/
#define SPI_MOSI 				/*13*/
#define SPI_MISO				7/*12*/
#define SPI_CLK					6/*11*/
#define SPI_SS_LEFT_MOTOR_ENCODER		5/*10*/
#define SPI_SS_RIGHT_MOTOR_ENCODER		4/*9*/
#define BITBANG_SPI 1

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
int dl, dr;
unsigned int dt;
/* MOTOR ENCODERS */
struct motorEncoderData current = {}; // TODO: rename these
struct motorEncoderData previous = {};
//double heading = 0;
// Settings
unsigned long lastsendtime;
int interog_dl = 100;
int sendMode = PULL;
int sendType = SEND_DTICK;
long rx_packetnum;
long tx_packetnum;

long int global_time;
long int arduino_time;


//long unsigned int reply_dtick_packet_num[50];
reply_dtick_t reply_dtick_packet_store[50];
int dtick_packet_store_pos;

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
	
	lastsendtime = millis();
	global_time = millis();
	
	rx_packetnum = 1;
	tx_packetnum = 1;
	dtick_packet_store_pos = 0;
}

void loop(void) {

	// TODO: don't do this if data is bad
	//previous = current;
	//readMotorEncoders(&current);

	//calcDelta();

	readSerial();
}

void calcDelta(void){
	dl = delta(current.leftMotorTick, previous.leftMotorTick, 100, TOTAL_ENCODER_TICKS - 1,  LEFT_MOTOR_ENCODER_DIRECTION); 
	dr = delta(current.rightMotorTick, previous.rightMotorTick, 100, TOTAL_ENCODER_TICKS - 1, RIGHT_MOTOR_ENCODER_DIRECTION);

	if( current.time > previous.time ){
		dt = (current.time - previous.time);
	}
	else{
		dt =  ( ((long)current.time) + 65536) - previous.time;
	}
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
/*
void readSerial(void) {
	if (Serial.available() > 0) {

		incomingByte = Serial.read();

		if (incomingByte == 'r') {
			sendStatus();
			return;//return here to keep from also pushing a packet, if PUSH is set, and the timer expired.
		} else if (incomingByte == 'w') {
			while (Serial.available()<2){}  // TODO: add timeout
			byte variableNumber = Serial.read();
			byte variableValue = Serial.read();
			setVariable(variableNumber, variableValue);
		} else if (incomingByte == 'i') {
			Serial.print("e");
		} else if(incomingByte == 'p') {
			unsigned long int packet_num;
			byte * bptr = (byte *)&packet_num;
			while (Serial.available()<4){}  // TODO: add timeout
			bptr[0] = Serial.read();
			bptr[1] = Serial.read();
			bptr[2] = Serial.read();
			bptr[3] = Serial.read();
			resend_packet(packet_num);
		} else {
			//error
		}
	}
	
	if(sendMode == PUSH){
		if(millis() < (lastsendtime + interog_dl)){//auto send timer expired
			sendStatus();
			lastsendtime = millis();
		}
	}
}
*/

void readSerial(void) {
	if (Serial.available() > 0) {
		while (Serial.available()<9){}// TODO: add timeout
			byte * bptr;
			long int intimestamp, inpacketnum;
			bptr = (byte *)&intimestamp;
			bptr[0] = Serial.read();
			bptr[1] = Serial.read();
			bptr[2] = Serial.read();
			bptr[3] = Serial.read();

			bptr = (byte *)&inpacketnum;
			bptr[0] = Serial.read();
			bptr[1] = Serial.read();
			bptr[2] = Serial.read();
			bptr[3] = Serial.read();

		incomingByte = Serial.read();

		if (incomingByte == 'r') {
			sendStatus();
			return;//return here to keep from also pushing a packet, if PUSH is set, and the timer expired.

		} else if (incomingByte == 'w') {
			while (Serial.available()<2){}  // TODO: add timeout
			byte variableNumber = Serial.read();
			byte variableValue = Serial.read();
			setVariable(variableNumber, variableValue);

		} else if (incomingByte == 'i') {
			long int timestamp =  global_time + millis() - arduino_time;
			unsigned long int packetnum = tx_packetnum;
			tx_packetnum++;
			serialPrintBytes(&timestamp, sizeof(long));
			serialPrintBytes(&packetnum, sizeof(long));
			Serial.print('e');

		} else if(incomingByte == 'p') {
			//long int timestamp =  global_time + millis() - arduino_time;
			unsigned long int packet_num;
			byte * bptr = (byte *)&packet_num;
			while (Serial.available()<4){}  // TODO: add timeout
			bptr[0] = Serial.read();
			bptr[1] = Serial.read();
			bptr[2] = Serial.read();
			bptr[3] = Serial.read();
			resend_packet(packet_num);

		} else {
			//error
		}
	}
	
	if(sendMode == PUSH){
		if(millis() < (lastsendtime + interog_dl)){//auto send timer expired
			sendStatus();
			lastsendtime = millis();
		}
	}
}

//TODO: make this use an array
void setVariable(byte num, byte val) {
	switch (num) {
		case PUSHPULL:
			sendMode = val;
			break;
		case RET_T:
			sendType = val;
			break;
		case INTEROG_DL:
//TODO: keep val in sensible boundaries
/*			if(val < XXX){
				val = XXX;
			}
*/
			interog_dl = val;
			break;
		case SETCLK:
			{			
			unsigned long int mills;
			byte * bptr = (byte *)&mills;
			while (Serial.available()<3){}  // TODO: add timeout
			bptr[0] = val;
			bptr[1] = Serial.read();
			bptr[2] = Serial.read();
			bptr[3] = Serial.read();
			global_time = mills;
			arduino_time = millis();
			break;
			}
/*
		case RESENDPKT:
		{
			unsigned long int packet_num;
			byte * bptr = (byte *)&packet_num;
			bptr[0] = val;
			bptr[1] = Serial.read();
			bptr[2] = Serial.read();
			bptr[3] = Serial.read();

			resend_packet(packet_num);
		}
*/
		default:
			//error
			break;
	}
	Serial.println(num);  //TODO: check for errors
}

void sendStatus() {
	//serialPrintBytes(&(current.leftMotorTick), sizeof(int));  // only 9 bits are used
	//serialPrintBytes(&(current.rightMotorTick), sizeof(int));
	//serialPrintBytes(&heading, sizeof(double));
	//serialPrintBytes(packetnum, sizeof(long));

	if(sendType == SEND_DTICK){
		reply_dtick_t packet;
		packet.timestamp =  global_time + millis() - arduino_time;
		packet.packetnum = tx_packetnum;
		tx_packetnum++;
		//packet.dl = dl;
		//packet.dr = dr;
//		packet.dt = dt;
		packet.dl = 0xDEAD;
		packet.dr = 0xFACE;
		packet.dt = 0xBEEF;
	
		serialPrintBytes(&packet, sizeof(packet));

		if(dtick_packet_store_pos < 50){
			reply_dtick_packet_store[dtick_packet_store_pos] = packet;
			dtick_packet_store_pos++;
		}
		else
		{
			dtick_packet_store_pos = 0;
			reply_dtick_packet_store[dtick_packet_store_pos] = packet;
		}

	}
	else if(sendType == SEND_CURRENT){

	}
/*
	//send packet num
	serialPrintBytes(&tx_packetnum, sizeof(long));
	tx_packetnum++;
	//send time
	long int timestamp = global_time + millis() - arduino_time;
	serialPrintBytes(&timestamp, sizeof(long));
	if(sendType == SEND_DTICK){
		serialPrintBytes(&dl, sizeof(int));
		serialPrintBytes(&dr, sizeof(int));
		serialPrintBytes(&dt, sizeof(unsigned int));
	}
	else if(sendType == SEND_CURRENT){

	}
*/
	//send checksum

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

void resend_packet(long unsigned int num){
	for(int i = 0; i < 50; i++ ){
		if(reply_dtick_packet_store[i].packetnum == num){
			serialPrintBytes(&reply_dtick_packet_store[i], sizeof(reply_dtick_t));
			return;
		}
		//else{

		//}
	}
	reply_dtick_t packet;
	packet.timestamp =  0xFFFFFFFF;
	packet.packetnum = tx_packetnum;
	tx_packetnum++;
	packet.dl = 0x0000;
	packet.dr = 0x0000;
	packet.dt = 0x0000;
	serialPrintBytes(&packet, sizeof(reply_dtick_t));
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

