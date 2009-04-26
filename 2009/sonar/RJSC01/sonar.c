//FIX THIS: SDEV_ADDRSET function has a strange bug. if I2craw is used, messes up all communication. ???


#include "sonar.h"
#include "twi.h"
#define NULL 0

//===========Init Vars================
int _STEPFREQ = 14; 		//_Step Frequency_
int _TOTALSTEPS = 1;		//_Total Steps_
int _ECHONUM = 1;		//_Echo Number_
int _TOTALDEV = 0;		//_Total Number of Devices_
byte proc_buff[5];		//SonDev Processor Buffer
SONDEV sonardevice[16];		//State Array of each Device
unsigned long ltime = 0;	//Timer for step gaps
byte cstep = 0xFF;		//Current Step Value, init as -1
byte pingFlag = 0;		//Ping Flag
//====================================

void SonDev_Start(int num, int initGain, int initMRange, int freq){
	int i;
	twi_init();	
	//initialize sonar structures
	for(i=0;i<16;i++){
		if(i<num){
			sonardevice[i].addr = 112+i;
			sonardevice[i].gain = initGain;
			sonardevice[i].mrange = initMRange;
			sonardevice[i].seqid = i;
			sonardevice[i].flags = (FLAG_ACTIVE_SET | FLAG_GEDIT_SET | FLAG_MREDIT_SET);
			SetGain((byte)sonardevice[i].addr,(byte)sonardevice[i].gain);
			SetMaxRange((byte)sonardevice[i].addr,(byte)sonardevice[i].mrange);
		}else{
			sonardevice[i].flags = 0;
		}
	}
	_TOTALDEV = num;
	_TOTALSTEPS = num;
	_STEPFREQ = freq;
}

void SonDev_Run(long unsigned int (*millis_fxn)()){ //<< having difficulties directly including source files to handle this; must import through fxn call
	int i,j;
	//check for all devices that want to change gain, or change mrange
	for(j=0;j<_TOTALDEV;j++){
		if(sonardevice[j].flags & FLAG_ACTIVE_SET){
			//gain edit
			if(sonardevice[j].flags & FLAG_GEDIT_SET){
				SetGain(sonardevice[j].addr,sonardevice[j].gain);
				sonardevice[j].flags &= ~FLAG_GEDIT_SET;
			}
			//mrange edit
			if(sonardevice[j].flags & FLAG_MREDIT_SET){
				SetMaxRange(sonardevice[j].addr,sonardevice[j].mrange);
				sonardevice[j].flags &= ~FLAG_MREDIT_SET;
			}	
		}
	}
	//Process Jobs, Ping
	for(i=0;i<=_TOTALSTEPS;i++){
		for(j=0;j<_TOTALDEV;j++){
			if(sonardevice[j].seqid == i && (sonardevice[j].flags & FLAG_ACTIVE_SET)){
			//device is on this step and active
				if(sonardevice[j].flags & FLAG_PING_SET){
				//ping requested
					if(cstep == 0xFF || i == cstep){
					//if cstep is reset or is set to this step
						StartRange(sonardevice[j].addr);
						sonardevice[j].flags &= ~FLAG_PING_SET;
						pingFlag=1;
						cstep = i;
					}
				}
								
			}	
		}
	}
	//Process Jobs, Read
	if(cstep ==_TOTALSTEPS){ 
		for(j=0;j<_TOTALDEV;j++){
			if(sonardevice[j].flags & FLAG_READ_SET){
				GetRange(sonardevice[j].addr,_ECHONUM, sonardevice[j].rdata);
				sonardevice[j].flags &= ~FLAG_READ_SET;
			}
		}
	}
	//Handle cstep
	if(!pingFlag){
		cstep++;
		ltime = millis_fxn();
	} else {
		if((millis_fxn()-ltime)>(1000/_STEPFREQ)){
			cstep++;
			ltime = millis_fxn();
			pingFlag = 0;
		}
	}
	if(cstep > _TOTALSTEPS){
		cstep == 0xFF;
	}

}

void SonDev_Process(byte* data, void (*sendpktfxn)(char cmd, int dataSize, byte* data)){
	switch(data[0]){
		//Ping Individual Device
		case SPING_IND:{
			byte index = data[1];
			sonardevice[index].flags |= (FLAG_PING_SET | FLAG_READ_SET);
			(*sendpktfxn)('s',0,NULL);
			break;
		}
		//Read Individual Device
		case SREAD_IND:{
			byte index = data[1];
			byte readdata[2];
			if(sonardevice[index].flags & FLAG_READ_SET){
				//Range has not been read yet; send -1 
				readdata[0] = 0xFF;
				readdata[1] = 0xFF;
				(*sendpktfxn)('s',2,readdata); 
			} else {
				//Read range back to user
				readdata[0] = sonardevice[index].rdata[0];
				readdata[1] = sonardevice[index].rdata[1];
				(*sendpktfxn)('s',2,readdata);
			}
			break;	
		}
		//Ping All Devices (in set order)
		case SPING_ALL:{
			byte i;
			for(i=0;i<_TOTALDEV;i++){
				if(sonardevice[i].flags & FLAG_ACTIVE_SET){
					sonardevice[i].flags |= (FLAG_PING_SET | FLAG_READ_SET);
				}
			}
			(*sendpktfxn)('s',0,NULL);
			break;	
		}
		//Import Readings from All Devices (in set order)
		case SREAD_ALL:{
			byte readdata[16*3+1];
			byte i,count = 0,readflag = 0;
			//Check for any unfinished processes
			for(i=0; i<_TOTALDEV; i++){
				if((sonardevice[i].flags & FLAG_READ_SET)){
					readflag = 1;
				}
			}
			//Find and Send range values
			if(!readflag){
				for(i=0; i<_TOTALDEV; i++){
					if(sonardevice[i].flags & FLAG_ACTIVE_SET){
						readdata[0+count*3] = i;
						readdata[1+count*3] = sonardevice[i].rdata[0];
						readdata[2+count*3] = sonardevice[i].rdata[1];
						count++;
						if(sonardevice[i].flags & FLAG_CONT_SET){
							sonardevice[i].flags |= (FLAG_PING_SET | FLAG_READ_SET);
						}
					}
				}
			}
			if(readflag) readdata[0] = 0xFF;
			readdata[count*3] = 0xFF;//'terminator byte' ... like the \0 in a string
			(*sendpktfxn)('s',(readflag) ? (1):(count*3+1),readdata);
			break;
		}
		//Set Gain for All Devices
		case SET_GAIN_ALL:{
			byte i;
			byte gainval = data[1];
			for(i=0;i<_TOTALDEV;i++){
				if(sonardevice[i].flags & FLAG_ACTIVE_SET){
					sonardevice[i].flags |= FLAG_GEDIT_SET;
					sonardevice[i].gain = gainval;
				}
			}
			(*sendpktfxn)('s',0,NULL);
			break;
		}
		//Set Gain for Individual Device
		case SET_GAIN_IND:{
			byte index = data[1];
			byte gainval = data[2];
			sonardevice[index].flags |= FLAG_GEDIT_SET;
			sonardevice[index].gain = gainval;
			(*sendpktfxn)('s',0,NULL);
			break;
		}
		//Get Gain Value from Individual Device
		case GET_GAIN_IND:{
			byte index = data[1];
			byte gainval = sonardevice[index].gain;
			(*sendpktfxn)('s',1,&gainval);
			break;
		}
		//Set Maximum Range for All Devices
		case SET_MRANGE_ALL:{
			byte i;
			byte mrangeval = data[1];
			for(i=0;i<_TOTALDEV;i++){
				if(sonardevice[i].flags && FLAG_ACTIVE_SET){
					sonardevice[i].flags |= FLAG_MREDIT_SET;
					sonardevice[i].mrange = mrangeval;
				}
			}
			(*sendpktfxn)('s',0,NULL);
			break;
		}
		//Set Maximum Range for Individual Device
		case SET_MRANGE_IND:{
			byte index = data[1];
			byte mrangeval = data[2];
			sonardevice[index].flags |= FLAG_MREDIT_SET;
			sonardevice[index].mrange = mrangeval;
			(*sendpktfxn)('s',0,NULL);
			break;
		}
		//Get Maximum Range for Individual Device
		case GET_MRANGE_IND:{
			byte index = data[1];
			byte mrangeval = sonardevice[index].mrange;
			(*sendpktfxn)('s',1,&mrangeval);
			break;
		}
		//Set Step Frequency for System
		case SET_FREQ:{
			byte freqval = data[1];
			_STEPFREQ = freqval;
			(*sendpktfxn)('s',0,NULL);
			break;
		}
		//Set Total Steps for System
		case SET_STEP_TOT:{
			byte steptotnum = data[1];
			_TOTALSTEPS = steptotnum;
			(*sendpktfxn)('s',0,NULL);	
			break;
		}
		//Set Step Value for Individual Device
		case SET_STEP_IND:{
			byte index = data[1];						
			byte stepval = data[2];
			sonardevice[index].seqid = stepval;
			(*sendpktfxn)('s',0,NULL);
			break;
		}
		//Set Individual Device Active
		case SET_ACTIVE_IND:{
			byte index = data[1];
			byte activeval = data[2];
			(activeval) ? (sonardevice[index].flags |= FLAG_ACTIVE_SET) : (sonardevice[index].flags &= ~FLAG_ACTIVE_SET);
			(*sendpktfxn)('s',0,NULL);
			break;
		}
		//Get Active Value of Individual Device
		case GET_ACTIVE_IND:{
			byte index = data[1];
			byte activeval = sonardevice[index].flags & FLAG_ACTIVE_SET;
			(*sendpktfxn)('s',1,&activeval);
			break;
		}
		//Get Step Value of Individual Device
		case GET_STEP_IND:{
			byte index = data[1];
			byte stepval = sonardevice[index].seqid;
			(*sendpktfxn)('s',1,&stepval);
			break;
		}
		//Get Step Frequency Value for System
		case GET_FREQ:{
			(*sendpktfxn)('s',1,(byte*)&_STEPFREQ);
			break;
		}
		//Get Step Total Value for System
		case GET_STEP_TOT:{
			(*sendpktfxn)('s',1,(byte*)&_TOTALSTEPS);
			break;
		}
		//Initialize System (MUST be called to run!)
		case START_SONDEV:{
			byte num = data[1];
			byte gain = data[2];
			byte mrange = data[3];
			byte freq = data[4];
			SonDev_Start((int)num, (int)gain, (int)mrange, (int)freq);
			(*sendpktfxn)('s',0,NULL);
			break;
		}	
		// Autopilot mode
		case SET_AUTO_ALL:{
			byte onoff = data[1];
			byte i;
			if(onoff){
				for(i=0;i<_TOTALDEV;i++){
					if(sonardevice[i].flags & FLAG_ACTIVE_SET){
						sonardevice[i].flags |= (FLAG_PING_SET | FLAG_READ_SET | FLAG_CONT_SET);
					}
				}
			} else {
				for(i=0;i<_TOTALDEV;i++){
					if(sonardevice[i].flags & FLAG_ACTIVE_SET){
						sonardevice[i].flags &= (~FLAG_CONT_SET);
					}
				}
			}
			(*sendpktfxn)('s',0,NULL);
			break;
		}
		//otherwise
		default:{
			//Send null packet to prevent hanging when cmd not recognized
			(*sendpktfxn)('s',0,NULL);
			break;
		}		
	}
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
	*(srdata+0) = 0x01;
	*(srdata+1) = GainVal;
	twi_writeTo((uint8_t)Addr,srdata,2,0);
	free(srdata);
}

void GetRange(int Addr, int EchoNum, byte* rdat){
	//Get Range Val from SonDev
	int val;
	uint8_t* srdata = (uint8_t*)calloc(1,sizeof(uint8_t));
	*(srdata+0) = EchoNum*2;
	twi_writeTo((uint8_t)Addr,srdata,1,0);
	free(srdata);
	twi_readFrom((uint8_t)Addr, rdat,2);
}

byte Range_MmToBin(int val){
	//Put Value through Alg (as specified by SRF08 Datasheet)
	return((byte)((val-43.0)/43.0)+1);
}
