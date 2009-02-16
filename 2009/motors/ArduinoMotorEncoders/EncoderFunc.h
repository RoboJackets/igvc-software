#ifndef ENCODERFUNC_H_
#define ENCODERFUNC_H_

#include "../../arduino/DataPacketStructs.hpp"

/* Reset TCNT1 */
void resetTime();

/* Read TCNT1 */
unsigned int getTime();

void readMotorEncoders(struct motorEncoderData *data);

void serialPrintBytes(void *data, int numBytes);

reply_dtick_t getEncoderStatus();
reply_current_t getCurrentStatus();

void calcDelta(void);

int delta(int p2, int p1, int maxDiff, int maxVal, int dir);

void readSerial(void);

int SPIReadInt(int inputPin, int slaveSelectPin, int clockPin);

int convertMotorEncoderFormat(unsigned int data);

void setVariable(byte num, byte val);

//resend a packet & incr the tx counter
void resend_packet(unsigned long num);

//save a packet
void savePacket(header_t head, byte * msg);

//Set the watchdog timer and loop (untested)
void softReset(void);

//loop and flash pin 13 every second
void hangAndFlash13(void);

//Send len bytes from the pointer
bool serialReadBytesTimeout(int len, byte * msg);

//gen a timestamp
void genTimestamp(long * sec, long * usec);
#endif // ENCODERFUNC_H_
