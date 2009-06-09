#include "MotorEncoders.h"
#include <cstdlib>
#include <sys/time.h>
#include <time.h>
#include <sys/timex.h>
#include <string.h>
#include <cmath>

typedef unsigned char byte;

MotorEncoders::MotorEncoders(void)
{

	//unsigned int rx_packetnum = 1;
	//unsigned int tx_packetnum = 1;
	arduinoInterface.initLink('e');
}

//true on error
bool MotorEncoders::getInfo(encoder_reply_t& out_data)
{
	DataPacket status;

	//arduinoInterface.getStatus(status);

	bool ret = arduinoInterface.sendCommand(ARDUINO_GETSTATUS_CMD, NULL, 0);

	byte cmdout;
	byte* out_data_rx = NULL;


	if (!arduinoInterface.recvCommand(cmdout, out_data_rx))
	{
		memcpy(&out_data, out_data_rx, sizeof(encoder_reply_t));
		delete[] out_data_rx;
	}
	else
	{
		return true;
	}

	return(false);
}

bool MotorEncoders::getInfo_class(byte*& out_data_rx)
{
	//EncoderData status;
	//status->data = new byte[sizeof(DataPacket::encoder_reply_t)];
	//bool ret = arduinoInterface.getStatus(*out_status);
	bool ret = arduinoInterface.sendCommand(ARDUINO_GETSTATUS_CMD, NULL, 0);


	byte cmdout;
	ret &= arduinoInterface.recvCommand(cmdout, out_data_rx);

	//status.setDataPointer(data);
	//status.packnum = rx_packetnum;
	//rx_packetnum++;
	//return(status);
	return(ret);
}

/*
EncoderPacket MotorEncoders::getDeltas(){
	EncoderPacket data;

	long timestamp;
	long packetnum;

	if(MotorEncoders::func != SEND_DTICK){
		MotorEncoders::setFunc(SEND_DTICK);
		MotorEncoders::func = SEND_DTICK;
	}
	byte status[14];
	getStatus(status, 14);

	memcpy(&data.timestamp, status, 4)
	memcpy(&data.packetnum, status+4, 4)
	memcpy(&data.dl, status+10, 2);
	memcpy(&data.dr, status+12, 2);
	memcpy(&data.time, status+14, 2);

	return(data);
}
*/

double MotorEncoders::getHeading(void)
{
	//EncoderPacket data = MotorEncoders::getDeltas();
	encoder_reply_t data;
	MotorEncoders::getInfo(data);
	MotorEncoders::heading += ( ((double)(data.dr - data.dl)) * (double)RAD_PER_ENCODER_TICK * (double)WHEEL_RADIUS / (double)WHEEL_BASE );
	while (MotorEncoders::heading >= (2*M_PI))   // this could be done more effiecently
	{
		MotorEncoders::heading -= (2*M_PI);
	}

	while (heading < 0)
	{
		MotorEncoders::heading += (2*M_PI);
	}


	return(MotorEncoders::heading);
}

double MotorEncoders::getRotVel(void)
{
	//EncoderPacket data = MotorEncoders::getDeltas();
	encoder_reply_t data;
	MotorEncoders::getInfo(data);
	/*
		if(data.packetnum != rx_packetnum){
			int trynum = 0;
			do{
				resendPacket(rx_packetnum, &data, sizeof(data));
				if(data.packetnum == rx_packetnum){
					rx_packetnum++;
					break;
				}
				trynum++;
			}while((data.packetnum != rx_packetnum) && (trynum < 5))
			return(NaN);
		}
		else{
			rx_packetnum++;
		}
	*/
	double dt = data.dt / COUNTER_RATE;

	double wr = data.dr / dt;
	double wl = data.dl / dt;

	double w = (wr - wl)*WHEEL_RADIUS/WHEEL_BASE;

	return(w);
}

void MotorEncoders::setHeading(double heading)
{
	MotorEncoders::heading = heading;
//	setVar(HEADING, &heading, sizeof(double));
}

bool MotorEncoders::setFunc(int mode)
{
	return( arduinoInterface.setVar(MC_RET_T, &mode, sizeof(int)) );
}

bool MotorEncoders::setSendMode(int mode)
{
	return( arduinoInterface.setVar(MC_PUSHPULL, &mode, sizeof(int)) );
}

bool MotorEncoders::setLogFile(string str)
{
	logfile = str;
}

bool MotorEncoders::setArduinoClock()
{
	//int time =
	struct timeval time;
	struct timezone tz;
	gettimeofday(&time, &tz);

	unsigned char timedata[4];

	long int millis = time.tv_sec*1000 + ((long int)((double)time.tv_usec/1000));

	//memcpy(timedata, &millis, 4);

	return( arduinoInterface.setVar(MC_SETCLK, &millis, 4 ) );
}
bool MotorEncoders::setSendMode(int mode, int int_rt)
{
	bool a = arduinoInterface.setVar(MC_PUSHPULL, &mode, sizeof(int));
	bool b = arduinoInterface.setVar(MC_INTEROG_DL, &int_rt, sizeof(int));

	return(a && b);
}


