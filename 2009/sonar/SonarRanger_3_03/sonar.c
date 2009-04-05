#include "sonar.h"
#include "twi.h"


//===========Init Vars================
int _STEPFREQ = 14; 		//Step Frequency
int _TOTALSTEPS = 1;		//Total Steps
int _ECHONUM = 1;			//Echo Number
int _TOTALDEV = 0;			//Total Number of Devices
extern int dbg;
SONDEV* sonardevice;

/*void SetupSonar(){
	Wire.begin();	
}*/

byte Range_MmToBin(int val){
	//Put Value through Alg (as specified by SRF08 Datasheet)
	return((byte)((val-43.0)/43.0));
}


void SonDev_Start(int num, int initGain, int initMRange, int freq){
	int i;
	twi_init();	
	sonardevice = (SONDEV*)calloc(num,sizeof(SONDEV));
	//initialize sonar structures
	for(i=0;i<num;i++){
		sonardevice[i].addr = 112+i;
		sonardevice[i].gain = initGain;
		sonardevice[i].mrange = initMRange;
		sonardevice[i].seqid = i;
		sonardevice[i].flags = FLAG_ACTIVE_SET;
		SetGain((byte)sonardevice[i].addr,(byte)sonardevice[i].gain);
		SetMaxRange((byte)sonardevice[i].addr,(byte)sonardevice[i].mrange);
	}
	//possible memory leak around here!
	_TOTALDEV = num;
	_TOTALSTEPS = num;
	_STEPFREQ = freq;
	
}

void SonDev_Run(void (*waitfxn)(long unsigned int)){
	int i,j,pingFlag;
	//check for devices that want to ping
	for(i=0;i<_TOTALSTEPS;i++){
		pingFlag = 0;
		for(j=0;j<_TOTALDEV;j++){
			if(sonardevice[j].seqid == i && (sonardevice[j].flags & FLAG_ACTIVE_SET)){

				if(sonardevice[j].flags & FLAG_PING_SET){
					StartRange(sonardevice[j].addr);
					if(!(sonardevice[j].flags & FLAG_CONT_SET)){
						sonardevice[j].flags &= ~FLAG_PING_SET;
					}
					pingFlag=1;
				}
								
			}	
		}
		if(pingFlag)(*waitfxn)(1000/_STEPFREQ); //for some reason, a direct call to delay freezes arduino, using fxn pointer
	}
	//check for all devices that want to read,change gain, or change mrange
	for(j=0;j<_TOTALDEV;j++){
		if(sonardevice[j].flags & FLAG_ACTIVE_SET){
			//read
			if(sonardevice[j].flags & FLAG_READ_SET){
				GetRange(sonardevice[j].addr,_ECHONUM, sonardevice[j].rdata);
				sonardevice[j].flags &= ~FLAG_READ_SET;
			}
			
			if(sonardevice[j].flags & FLAG_GEDIT_SET){
				SetGain(sonardevice[j].addr,sonardevice[j].gain);
				sonardevice[j].flags &= ~FLAG_GEDIT_SET;
			}
			
			if(sonardevice[j].flags & FLAG_MREDIT_SET){
				SetMaxRange(sonardevice[j].addr,sonardevice[j].mrange);
				sonardevice[j].flags &= ~FLAG_MREDIT_SET;
			}
			
		}
	}
	
}

byte* SonDev_Process(byte* data){
	
}

void StartRange(int Addr){
	//Send Ping Command to SonDev
	uint8_t* srdata = (uint8_t*)calloc(2,sizeof(uint8_t));
	*(srdata+0) = 0x00;
	*(srdata+1) = 0x51;
	twi_writeTo((uint8_t)Addr,srdata,2,0);
	free(srdata);
}

void SetMaxRange(byte Addr, byte RangeVal){
	//Send Max Range Val on SonDev
	uint8_t* srdata = (uint8_t*)calloc(2,sizeof(uint8_t));
	*(srdata+0) = 0x02;
	*(srdata+1) = RangeVal;
	twi_writeTo((uint8_t)Addr,srdata,2,0);
	free(srdata);
}

void SetGain(byte Addr, byte GainVal){
	//Set Gain Val on SonDev	
	uint8_t* srdata = (uint8_t*)calloc(2,sizeof(uint8_t));
	*(srdata+0) = 0x02;
	*(srdata+1) = GainVal;
	twi_writeTo((uint8_t)Addr,srdata,2,0);
	free(srdata);
}

void GetRange(int Addr, int EchoNum, byte* rdat){
	//Send Request for Range from SonDev
	int val;
	uint8_t* srdata = (uint8_t*)calloc(1,sizeof(uint8_t));
	*(srdata+0) = EchoNum*2;
	twi_writeTo((uint8_t)Addr,srdata,1,0);
	free(srdata);
	
	//Get Range Val From SonDev
	//uint8_t* rxbuff = (uint8_t*)calloc(2,sizeof(uint8_t));
	twi_readFrom((uint8_t)Addr, rdat,2);
	//val = 10*((int)(*(rxbuff+1))|((*(rxbuff+0))<<8));
	//free(rxbuff);
	//return(val);
}
