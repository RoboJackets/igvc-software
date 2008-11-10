/***************************************
* Driver class for URG-04LX laser range finder
*
* by Kawaji Kouhei, 2006.
* modified by Sasaki, Brscic, 2007.
***************************************/
#include <cerrno>
#include <cstdlib>
#include <iostream>

#include "urg_laser.h"

using namespace std;

/**********************************************************
UrgLaser::UrgLaser()
  Constructor
***********************************************************/
UrgLaser::UrgLaser(){
	maxrange = 5600; //max range in [mm]
	minangle = -135; //min angle in [deg]
	maxangle = 135; //max angle in [deg]
	maxstep = 768;
}

/**********************************************************
UrgLaser::initialize(char* aDevName, int aSpeed)
  Initialization functions

 Input: aDevName -- device name (default = "/dev/ttyACM0")
        aSpeed -- communication speed (default = 19200bps)
***********************************************************/
void UrgLaser::initialize(char* aDevName, int aSpeed){
	// Open serial port (aSpeed bps)
	dev=openSerial(aDevName,aSpeed);
	if(dev==NULL){
		perror("Could not open serial port");
		exit(1);
	}
	int tests = startSensor(dev);
	if(tests<0){
		perror("Error in startSenssor");
		cerr << tests << endl;
		exit(1);
	}

	return;
}

/******************************************************
UrgLaser::openSerial(char *aDevName, int aSpeed)
  Open and set up specified serial port
	aDevNme [in] Serial port device file name
	aSpeed [in] Communication speed(bps)
	Return serial port file handle
	Returns NULL when failed to open.
******************************************************/
FILE* UrgLaser::openSerial(char *aDevName, int aSpeed){
	FILE* dev;

	dev=fopen(aDevName,"w+");
	if(dev==NULL) return NULL;	// invalid device file

	// setup parameters
	if(setupSerial(fileno(dev),aSpeed)) return dev;

	// fail
	perror("setting term parameters");
	fclose(dev);
	return NULL;
}

/******************************************************
UrgLaser::setupSerial(int aFd, int aSpeed)
	Parameter settings for serial port communication
	aFd [in] File descriptor to point serial port
	aSpeed [in] Communication speed (bps)
 	Retval 0 Failed, 1 Success
*******************************************************/
int UrgLaser::setupSerial(int aFd, int aSpeed){
	int speed;
	struct termios tio;

	speed=bitrate(aSpeed);
	if(speed==0)
	return 0;	// invalid bitrate

	tcgetattr(aFd,&tio);

	cfsetispeed(&tio,speed); // bitrate
	cfsetospeed(&tio,speed); // bitrate

	tio.c_cflag=(tio.c_cflag & ~CSIZE) | CS8; // data bits = 8bit

	tio.c_iflag&= ~( BRKINT | ICRNL | ISTRIP );
	tio.c_iflag&= ~ IXON;	// no XON/XOFF
	tio.c_cflag&= ~PARENB;	 // no parity
	tio.c_cflag&= ~CRTSCTS;	// no CTS/RTS
	tio.c_cflag&= ~CSTOPB;	 // stop bit = 1bit

	// Other
	tio.c_lflag &= ~( ISIG | ICANON | ECHO );

	// Commit
	if(tcsetattr(aFd,TCSADRAIN,&tio)==0) return 1;

	return 0;
}

/****************************************************************
UrgLaser::bitrate(int aBR)
	Communication port speed settingds
  	aBR [in] Declared for communication speed (9600-500000)
  	cfset[i|o] Returned value for speed function
	Returns B0 if failed
*****************************************************************/
speed_t UrgLaser::bitrate(int aBR){
	switch(aBR){
	case 9600:
		return B9600;

	case 19200:
		return B19200;

	case 38400:
		return B38400;

	case 57600:
		return B57600;

	case 115200:
		return B115200;

	case 230400:
		return B230400;

	case 460800:
		return B460800;

	case 500000:
		return B500000;

	default: // invalid bitrate
		return B0;
	}
}

/***********************************************
UrgLaser::startSensor(FILE* aPort)
	Set SCIP2.0 protocol and start the laser
************************************************/
int UrgLaser::startSensor(FILE* aPort){
	char req[8];
	int i;
	char echo[80];

	// ---- send SCIP2.0 protocol request ----
	sprintf(req,"SCIP2.0\n");
	i=fwrite(req,sizeof(char),strlen(req),aPort);
	// 'fwrite' error
	if(i==0) return -1;

	// echo message
	// 'fgets' error
	if(fgets(echo, sizeof(echo), aPort) == NULL) return -2;
	// check echo message (== "SCIP2.0")
	cerr << echo;

	fgets(echo, sizeof(echo), aPort);
	cerr << echo << endl;
	if ( (strncmp("0Ee",echo,3)==0) ){
		fgets(echo, sizeof(echo), aPort);
		cerr << "SCIP2.0 protocol already set" << endl;
	} else cerr << "Set SCIP2.0 protocol" << endl;

	// ---- get version ----
	sprintf(req,"VV\n");
	i=fwrite(req,sizeof(char),strlen(req),aPort);
	// 'fwrite' error
	if(i==0) return -1;

	// echo message
	// 'fgets' error
	if(fgets(echo, sizeof(echo), aPort) == NULL) return -2;
	// check echo message (== "BM____")
	cerr << endl << echo << ':';
	fflush(NULL);

	if ( (strncmp(req,echo,8)!=0) ) return -3;

	fgets(echo, sizeof(echo), aPort);
	cerr << echo;
	fgets(echo, sizeof(echo), aPort);
	cerr << echo;
	fgets(echo, sizeof(echo), aPort);
	cerr << echo;
	fgets(echo, sizeof(echo), aPort);
	cerr << echo;
	fgets(echo, sizeof(echo), aPort);
	cerr << echo;
	fgets(echo, sizeof(echo), aPort);
	cerr << echo;
	fgets(echo, sizeof(echo), aPort);


	// ---- start laser message ----
	sprintf(req,"BM\n");
	i=fwrite(req,sizeof(char),strlen(req),aPort);
	// 'fwrite' error
	if(i==0) return -1;

	// echo message
	// 'fgets' error
	if(fgets(echo, sizeof(echo), aPort) == NULL) return -2;
	// check echo message (== "BM____")
	cerr << endl << echo << ':';
	fflush(NULL);

	if ( (strncmp(req,echo,8)!=0) ) return -3;

	// status
	fgets(echo, sizeof(echo), aPort);
	cerr << echo;
	fgets(echo, sizeof(echo), aPort);

	return 1;

}

/****************************************
UrgLaser::~UrgLaser()
	Destructor
*****************************************/
UrgLaser::~UrgLaser(){
}

/****************************************
UrgLaser::closeSerial();
	Close sensor
*****************************************/
void UrgLaser::closeSerial(){
	int tests=stopSensor(dev);
	if(tests<0){
		perror("Error in stopSenssor");
		cerr << tests << endl;
	}
	fclose (dev);

	return;
}

/****************************************
UrgLaser::stopSensor(FILE* aPort)
	Stop the laser
*****************************************/
int UrgLaser::stopSensor(FILE* aPort){
	char req[8];
	int i;
	char echo[80];

	// ---- stop laser message ----
	sprintf(req,"QT\n");
	i=fwrite(req,sizeof(char),strlen(req),aPort);
	// 'fwrite' error
	if(i==0) return -1;

	// echo message
	// 'fgets' error
	if(fgets(echo, sizeof(echo), aPort) == NULL) return -2;
	// check echo message (== "QT____")
	cerr << endl << echo << ':';
	fflush(NULL);

	if ( (strncmp(req,echo,8)!=0) ) return -3;

	// status
	fgets(echo, sizeof(echo), aPort);
	cerr << echo;
	fgets(echo, sizeof(echo), aPort);

	return 1;
}

/****************************************************************************************
int UrgLaser::urgMakeScan(FILE* aPort, int aStart, int aEnd, int aSkip)
	Perform scan
	Params:
 		aPort [in] Serial port file handle where URG is connected
  		aStart [in] Starting scan step from (0-768)
  		aEnd [in] Ending scan step at (0-768)
  		aSkip [in] Number of steps to be merged when multiple points are combined(1-99)
  	Return: 0 if OK, -1 if failed
******************************************************************************************/
int UrgLaser::urgMakeScan(FILE* aPort, int aStart, int aEnd, int aSkip){
	char req[16];
	int *pt;
	char line[70];
	char *alldata=(char *)NULL;
	char *pt_all;
	int i, lenlin;


	sprintf(req,"GD%04d%04d%02d\n",aStart,aEnd,aSkip);
	i=fwrite(req,sizeof(char),strlen(req),aPort);
	if(i==0)
		return -1;
	// echo-back
	if(fgets(line,sizeof(line),aPort)==NULL) return -1;
	if(strcmp(req,line)!=0)	return -1;

	// status
	if(fgets(line,sizeof(line),aPort)==NULL) {
		return -1;
	}
	if(strcmp("00P\n",line)!=0)
	{
		return -1;
	}

	// Timestamp and sum
	if(fgets(line,sizeof(line),aPort)==NULL) {
		return -1;
	}

	// result
	pt=data;

	int count = 0;
	alldata =new char[3*((aEnd-aStart+1)/aSkip+1)];

	for(;;){

		if(fgets(line,sizeof(line),aPort)==NULL){
			return -1;
		}
		if(!strcmp("\n",line)){
			//end of result
			break;
		}

		// copy the data part into 'alldata'
		pt_all = (char *)(alldata+count*64);
		if ((lenlin = strlen(line) )== 66){
			strncpy(pt_all, line, 64);
		}else{
			strncpy(pt_all, line, lenlin-2);
			*(pt_all+lenlin-2)='\n';
		}
		count++;
	}

	pt_all = alldata;

	int cnt = 0;
	for(;;){
		cnt++;
		if (*(pt_all) == '\n') {
			break;
		}
		*(pt)=((((*(pt_all) - 0x30)<<6) + *(++pt_all) - 0x30)<<6) + *(++pt_all) - 0x30;
		pt++;
		pt_all++;

	}

	delete [] alldata;

	return 0;
}

