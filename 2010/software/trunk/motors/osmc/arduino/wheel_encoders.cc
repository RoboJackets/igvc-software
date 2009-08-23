#include "wheel_encoders.hpp"

unsigned int getTime()
{
	unsigned int time;
	//TCNT1 is a 16 bit timer/counter ~ pg 121
	time = TCNT1L;//how often does this tick? tests say clkio = F_CPU.
	time |= TCNT1H << 8;
	return(time);
}

void calcDelta(struct motorEncoderData current, struct motorEncoderData previous, int *dl, int *dr, long *dt)
{
	*dl = delta(current.leftMotorTick, previous.leftMotorTick, 100, TOTAL_ENCODER_TICKS - 1,  LEFT_MOTOR_ENCODER_DIRECTION); 
	*dr = delta(current.rightMotorTick, previous.rightMotorTick, 100, TOTAL_ENCODER_TICKS - 1, RIGHT_MOTOR_ENCODER_DIRECTION);

	if( current.time > previous.time ){
		*dt = (current.time - previous.time);
	}
	else{
		*dt =  ( ((long)current.time) + 65536) - previous.time;
	}
}

void readMotorEncoders(struct motorEncoderData *data)
{

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
int delta(int p2, int p1, int maxDiff, int maxVal, int dir)
{
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
int convertMotorEncoderFormat(unsigned int data)
{
	//XX: trash data does not match the spec
	if( (data & 0x0078) | ~(data | 0x3F7F) ) {
		//Serial.print("Frame error\n");
		//error
		return(0);
	} else {
		return( (data & 0x0007) | ((data & 0x3F00) >> 5) );
	}
}
