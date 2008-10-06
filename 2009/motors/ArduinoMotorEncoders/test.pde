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
		
	lastsendtime = millis();
	global_time = millis();
	
	rx_packetnum = 1;
	tx_packetnum = 1;
	dtick_packet_store_pos = 0;
}

void loop(void) {

	Serial.print(sizeof(reply_dtick_t));
	Serial.print("\n");
	delay(1000);
	//readSerial();
}


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
		packet.dl = 0xFACE;
		packet.dr = 0xFACE;
		packet.dt = 0xFACE;
	
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



void resend_packet(long unsigned int num){
	for(int i = 0; i < 50; i++ ){
		if(reply_dtick_packet_store[i].packetnum == num){
			serialPrintBytes(&reply_dtick_packet_store[i], sizeof(reply_dtick_t));
			return;
		}
		//else{

		//}
	}
}


void serialPrintBytes(void *data, int numBytes) {
	for (int i = 0; i < numBytes; i++) {
		Serial.print(((unsigned char *)data)[i], BYTE);
	}
}


