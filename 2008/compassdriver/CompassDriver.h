#ifndef COMPASS_DRIVER_H
#define COMPASS_DRIVER_H

/*no errors reporting from anything yet, need to decide big of little endian,
 set in config and driver -- right now assuming little endian*/
/*functions known incomplete: CompassDriver, ~CompassDriver, CompassDriver::spiSend, CompassDriver::spiGet, GetModInfo, GetData, SetDataComponents,
SetConfig, GetConfig, SaveConfig, StartCal, StopCal, need to check if endianness matters*/

//setcaldata, getcaldata, getdata,setconfig


#include "compassTypes.h"
#include "BitbangSPI.h"
#include <cstdio>

//debug opts -- not set yet
//#define DEBUG_MESG

class CompassDriver {
	private://do these need to be static?

		//for future sanity checking
		int datalength;
		int datapackcount;
		//uint_8 lastcommand;//moved to spidriver
	
		//spi driver instance -- should dynamic allocation be done instead, like in the player driver?
		BitbangSPI spidriver;
	
	public:
		//CompassDriver(ConfigData configdata, DataRespType respdata);
		CompassDriver(ConfigData configdata, DataTypeReq respdata);
		~CompassDriver();

		int CompassSend(uint_8 * data, int size);
		ModInfoResp GetModInfo(void);
		compassData GetData(void);
		//int SetDataComponents(DataRespType datatypewanted);
		int SetDataComponents(DataTypeReq datatypewanted);
		int SetConfig(uint_8 config_id, uint_8 * convigval);//this needs to be able to accept bool, uint_8, and Float32, but i think it will take just bytes
		//uint_8 * GetConfig(uint_8 config_id);//removed in favor of GetAllConfig
		int SaveConfig(void);
		int StartCal(void);
		int StopCal(void);
		CalDataResp GetCalData(void);
		int SetCalData(CalDataResp caldata);
		int SetAllConfig(ConfigData configdata);
		ConfigData GetAllConfig(uint_8 config_id);
};


int CompassDriver::CompassSend(uint_8 * data, int size) {

	uint_8 senddata[size+2];
	senddata[0] = sync_flag;
	senddata[size+1] = terminator;
	for(int i=0;i<size; i++) {
		senddata[i+1] = data[i];
	}
	spidriver.spiSend(senddata, size+2);//send data wrapped with sync_flag and terminator
	return(0);
}

CompassDriver::CompassDriver(ConfigData configdata, DataTypeReq respdata) {
	//CompassDriver::spiinit();
	printf("seting config -- constructor\n");
	SetAllConfig(configdata);
	printf("done seting config -- constructor\n");
	printf("setting data componets -- constructor\n");
	SetDataComponents(respdata);
	printf("done setting data componets -- constructor\n");
	
}

int CompassDriver::SetAllConfig(ConfigData configdata) {
	float_u8 tempflt;
	tempflt.flt = configdata.declination;

	SetConfig(declination, tempflt.u8);
	SetConfig(true_north, &configdata.truenorth);
	SetConfig(calsamplefreq, &configdata.calsamplefreq);
	SetConfig(samplefreq, &configdata.samplefreq);
	SetConfig(period, &configdata.period);
	SetConfig(bigendian, &configdata.bigendian);
	SetConfig(dampingsize, &configdata.dampingsize);
}

CompassDriver::~CompassDriver() {
	printf("CompassDriver destructor\n");
	//CompassDriver::spioff();
	//delete spidriver;
}
	
ModInfoResp CompassDriver::GetModInfo(void) {
	char_u8 tempchr;
	uint_8 data = get_mod_info;
	CompassSend(&data,1);
	uint_8 resp[11];
	spidriver.spiGet(resp, 11);
	ModInfoResp reply;
		for(int i = 2;i<6;i++) {
			tempchr.u8 = resp[i];
			reply.module_type[i-2] = tempchr.chr;
		}
		reply.module_type[4] = '\0';//probably usefull
		
		for(int i = 6;i<10;i++) {
			tempchr.u8 = resp[i];
			reply.firmware_version[i-6] = tempchr.chr;
		}	       
		reply.firmware_version[4] = '\0';
	return(reply);
}

/*
//code needs to be rewritten so this can be removed
bool checkbitset(short int foo, int i){//from http://www.cs.umd.edu/class/spring2003/cmsc311/Notes/BitOp/bitI.html
	short int mask = 1 << i;
	return(mask & foo);
}
//
*/

int CompassDriver::SetDataComponents(DataTypeReq datatypewanted) {
	uint_8 data[11];//max size of config + config count + command, excluding header (CompassSend adds it).  the number of bits sent is determind by j, so the unused bits should be ignored
	
	CompassDriver::datapackcount = 0;
	CompassDriver::datalength = 0;
	int j = 2;

	if(datatypewanted.xraw){
		data[j] = XRaw;
		j++;
		datapackcount += 1;//number of data types expected
		datalength += (4 + 1);//length of data + byte label
	}
	if(datatypewanted.yraw){
		data[j] = YRaw;
		j++;
		datapackcount += 1;
		datalength += (4 + 1);
	}
	if(datatypewanted.xcal){
		data[j] = XCal;
		j++;
		datapackcount += 1;
		datalength += (4 + 1);
	}
	if(datatypewanted.ycal){
		data[j] = YCal;
		j++;
		datapackcount += 1;
		datalength += (4 + 1);
	}
	if(datatypewanted.heading){
		data[j] = Heading;
		j++;
		datapackcount += 1;
		datalength += (4 + 1);
	}
	if(datatypewanted.magnitude){
		data[j] = Magnitude;
		j++;
		datapackcount += 1;
		datalength += (4 + 1);
	}
	if(datatypewanted.temperature){
		data[j] = Temperature;
		j++;
		datapackcount += 1;
		datalength += (4 + 1);
	}
	if(datatypewanted.distortion){
		data[j] = Distortion;
		j++;
		datapackcount += 1;
		datalength += (1 + 1);
	}
	if(datatypewanted.calstatus){
		data[j] = CalStatus;
		j++;
		datapackcount += 1;
		datalength += (1 + 1);
	}

	data[0] = set_data_components;//command to set data to recive
	data[1] = datapackcount;//count of data we expect
	
	CompassSend(data, j);
	return(0);
}
/*
int CompassDriver::SetDataComponents(DataRespType datatypewanted) {//input type of data to get, bitfield is named in compassTypes.h
	uint_8 data[11];//max size of config + config count + command, excluding header (CompassSend adds it).  the number of bits sent is determind by j, so the unused bits should be ignored
	


	//Generate data stream, and set private "datalength" to length of data to expect
	//const uint_8 dataresponsetype[9] = {XRaw, YRaw, XCal, YCal, Heading, Magnitude, Temperature, Distortion, CalStatus};//moved to compassTypes.h
	//const uint_8 dataresponsetypelength[9] = {4, 4, 4, 4, 4, 4, 4, 1, 1};//number of bytes data is
	datapackcount = 0;//reset to zero
	datalength = 0;
	int j = 2;


	for(int i=0;i<10;i++) {
		if (checkbitset(datatypewanted.sint,i)) {
			data[j] = dataresponsetype[i];
			j++;
			datapackcount +=1;//used te check getdata got correct num of packs later
			datalength += (dataresponsetypelength[i] + 1);//datalength = length of package + 1 header per package
		}

	}
	data[0] = set_data_components;//command to set data to recive
	data[1] = datapackcount;//count of data we expect
	
	CompassSend(data, j);
	return(0);
}
*/

compassData CompassDriver::GetData(void) {
	uint_8 data = get_data;
	CompassSend(&data,1);

	uint_8 dataresp[datalength+4];//we will be getting datalength + sync + data_resp + count + term
	spidriver.spiGet(dataresp, datalength+4);
		

	compassData datarespstruct;
	int i = 3;//first data type packet is byte 4 (3 in array)

	
	float_u8 tempflt;
	s32_u8 temps32;

	if(dataresp[2] == datapackcount) {//check for the number of bytes we expect to get
		for(int j = 0; j < datapackcount; j++) {

//could do a switch/case instead -- better?				
/*			switch(dataresp[i]){
				case XRaw:{
					temps32.u8[0] = dataresp[i+1];
					temps32.u8[1] = dataresp[i+2];
					temps32.u8[2] = dataresp[i+3];
					temps32.u8[3] = dataresp[i+4];
					datarespstruct.XRaw = temps32.s32;
					i += 5;//go forward 5 bytes to next data header
					break;
				}
			}
*/
			if(dataresp[i] == XRaw) {
				temps32.u8[0] = dataresp[i+1];
				temps32.u8[1] = dataresp[i+2];
				temps32.u8[2] = dataresp[i+3];
				temps32.u8[3] = dataresp[i+4];
				datarespstruct.XRaw = temps32.s32;
				i += 5;//go forward 5 bytes to next data header
			}
			else if(dataresp[i] == YRaw) {
				temps32.u8[0] = dataresp[i+1];
				temps32.u8[1] = dataresp[i+2];
				temps32.u8[2] = dataresp[i+3];
				temps32.u8[3] = dataresp[i+4];
				datarespstruct.YRaw = temps32.s32;
				i += 5;
			}
			else if(dataresp[i] == XCal) {
				tempflt.u8[0] = dataresp[i+1];
				tempflt.u8[1] = dataresp[i+2];
				tempflt.u8[2] = dataresp[i+3];
				tempflt.u8[3] = dataresp[i+4];
				datarespstruct.XCal = tempflt.flt;
				i += 5;
			}
			else if(dataresp[i] == YCal) {
				tempflt.u8[0] = dataresp[i+1];
				tempflt.u8[1] = dataresp[i+2];
				tempflt.u8[2] = dataresp[i+3];
				tempflt.u8[3] = dataresp[i+4];
				datarespstruct.YCal = tempflt.flt;
				i += 5;
			}
			else if(dataresp[i] == Heading) {
				tempflt.u8[0] = dataresp[i+1];
				tempflt.u8[1] = dataresp[i+2];
				tempflt.u8[2] = dataresp[i+3];
				tempflt.u8[3] = dataresp[i+4];
				datarespstruct.Heading = tempflt.flt;
				i += 5;
			}
			else if(dataresp[i] == Magnitude) {
				tempflt.u8[0] = dataresp[i+1];
				tempflt.u8[1] = dataresp[i+2];
				tempflt.u8[2] = dataresp[i+3];
				tempflt.u8[3] = dataresp[i+4];
				datarespstruct.Magnitude = tempflt.flt;
				i +=5;
			}
			else if(dataresp[i] == Temperature) {
				tempflt.u8[0] = dataresp[i+1];
				tempflt.u8[1] = dataresp[i+2];
				tempflt.u8[2] = dataresp[i+3];
				tempflt.u8[3] = dataresp[i+4];
				datarespstruct.Temperature = tempflt.flt;
				i += 5;
			}
			else if(dataresp[i] == Distortion) {
				datarespstruct.Distortion = dataresp[i+1];//says bool, but guessing send bytes at a time
				i += 2;
			}
			else if(dataresp[i] == CalStatus) {
				datarespstruct.CalStatus = dataresp[i+1];
				i += 2;
			}
			else {
				//handle the error somehow (data type bit didn't match)
				printf("unknown data type sent\n");
				return(datarespstruct);
			}
		}//end for loop

		return(datarespstruct);

	}//end if block
	else {//we didn't recieve the right number of bytes
		printf("wrong number of bytes sent");
		return(datarespstruct);
	}
}

int CompassDriver::SetConfig(uint_8 config_id, uint_8 * configval) {
	
	if (config_id == declination) {
		uint_8 data[6];
		data[0] = set_config;
		data[1] = config_id;
			data[2] = configval[0];
			data[3] = configval[1];
			data[4] = configval[2];
			data[5] = configval[3];
		CompassSend(data,6);
	}
	else {
		uint_8 data[3];
		data[0] = set_config;
		data[1] = config_id;
		data[3] = configval[0];
		CompassSend(data,3);
	}

	return(0);
}

ConfigData CompassDriver::GetAllConfig(uint_8 config_id) {

	ConfigData rxconfigdata;
	uint_8 data[2];
	float_u8 tempflt;
	for(uint_8 config_id = 1; config_id < 8; config_id++){
		data[0] = get_config;
		data[1] = config_id;
		CompassSend(data, 2);

		switch (config_id) {
			case declination: {
				uint_8 resp[8];
				spidriver.spiGet(resp, 8);

				tempflt.u8[0] = resp[3];
				tempflt.u8[1] = resp[4];
				tempflt.u8[2] = resp[5];
				tempflt.u8[3] = resp[6];
				rxconfigdata.declination = tempflt.flt;
				break;
			}
			case true_north: {
				uint_8 resp[5];
				spidriver.spiGet(resp, 5);
				rxconfigdata.truenorth = resp[4];
				break;
			}
			case calsamplefreq: {
				uint_8 resp[5];
				spidriver.spiGet(resp, 5);
				rxconfigdata.calsamplefreq = resp[4];
				break;
			}
			case samplefreq: {
				uint_8 resp[5];
				spidriver.spiGet(resp, 5);
				rxconfigdata.samplefreq = resp[4];
				break;
			}
			case period: {
				uint_8 resp[5];
				spidriver.spiGet(resp, 5);
				rxconfigdata.period = resp[4];
				break;
			}
			case bigendian: {
				uint_8 resp[5];
				spidriver.spiGet(resp, 5);
				rxconfigdata.bigendian = resp[4];
				break;
			}
			case dampingsize: {
				uint_8 resp[5];
				spidriver.spiGet(resp, 5);
				rxconfigdata.dampingsize = resp[4];
				break;
			}
		}
	}
	return(rxconfigdata);
}
/*
uint_8 * CompassDriver::GetConfig(uint_8 config_id) {
	uint_8 data[2];
	data[0] = get_config;
	data[1] = config_id;
	CompassSend(data, 2);

	float_u8 tempflt;
	if (config_id == declination) {
		uint_8 resp[8];
		spidriver.spiGet(resp, 8);


		//tempflt.u8[0] = resp[3];
		//tempflt.u8[1] = resp[4];
		//tempflt.u8[2] = resp[5];
		//tempflt.u8[3] = resp[6];
		//return(tempflt.flt);//this won't work because the func can't return a u8 and a float -- right? maybe dump to a struct and return that
		uint_8 out[4] = {resp[3],resp[4],resp[5],resp[6]};
		return(out);//returns the second element
	}
	else {
		uint_8 resp[5];
		spidriver.spiGet(resp, 5);
		return(&resp[4]);//returns the second element, assuming it is a byte.
	}
	
}
*/

int CompassDriver::SaveConfig(void) {
	uint_8 data = save_config;
	CompassSend(&data,1);
	return(0);
}

int CompassDriver::StartCal(void) {
	uint_8 data = start_cal;
	CompassSend(&data,1);
	return(0);//something about wanting to read XRaw and YRaw??? needs to be set before this is run

}

int CompassDriver::StopCal(void) {
	uint_8 data = stop_cal;
	CompassSend(&data,1);
	return(0);//need to SaveConfig for this to be permanent
}

CalDataResp CompassDriver::GetCalData(void) {
	uint_8 data = get_cal_data;
	CompassSend(&data,1);
	CalDataResp reply;
	uint_8 resp[28];//is this right? needs to be total length of cal data from device.  (4 x sint_32, 2 x float32) = 24 bytes, + count + sync + term + cal_data_resp
	spidriver.spiGet(resp, 28);//getting array pf bytes from compass


	float_u8 tempflt;
	s32_u8 temps32;
	//sorting and converting bytes
	//if(resp[2] == 24) {//can add this to see if header is correct
		temps32.u8[0] = resp[3];
		temps32.u8[1] = resp[4];
		temps32.u8[2] = resp[5];
		temps32.u8[3] = resp[6];
		reply.XOffset = temps32.s32;

		temps32.u8[0] = resp[7];
		temps32.u8[1] = resp[8];
		temps32.u8[2] = resp[9];
		temps32.u8[3] = resp[10];
		reply.YOffset = temps32.s32;

		temps32.u8[0] = resp[11];
		temps32.u8[1] = resp[12];
		temps32.u8[2] = resp[13];
		temps32.u8[3] = resp[14];
		reply.XGain = temps32.s32;

		temps32.u8[0] = resp[15];
		temps32.u8[1] = resp[16];
		temps32.u8[2] = resp[17];
		temps32.u8[3] = resp[18];
		reply.YGain = temps32.s32;

		tempflt.u8[0] = resp[19];
		tempflt.u8[1] = resp[20];
		tempflt.u8[2] = resp[21];
		tempflt.u8[3] = resp[22];
		reply.phi = tempflt.flt;

		tempflt.u8[0] = resp[23];
		tempflt.u8[1] = resp[24];
		tempflt.u8[2] = resp[25];
		tempflt.u8[3] = resp[26];
		reply.CalibrationMagnitude = tempflt.flt;

	//}
	//else{
		//handle error
		//return(error);
	//}
	
	return(reply);//return structure
}

int CompassDriver::SetCalData(CalDataResp caldata) {
	uint_8 data[26];

	data[0] = set_cal_data;//command
	data[1] = 24;//byte count

	float_u8 tempflt;
	s32_u8 temps32;
	temps32.s32 = caldata.XOffset;
		data[2] = temps32.u8[0];
		data[3] = temps32.u8[1];
		data[4] = temps32.u8[2];
		data[5] = temps32.u8[3];
	
	temps32.s32 = caldata.YOffset;
		data[6] = temps32.u8[0];
		data[7] = temps32.u8[1];
		data[8] = temps32.u8[2];
		data[9] = temps32.u8[3];

	temps32.s32 = caldata.XGain;
		data[10] = temps32.u8[0];
		data[11] = temps32.u8[1];
		data[12] = temps32.u8[2];
		data[13] = temps32.u8[3];

	temps32.s32 = caldata.YGain;
		data[14] = temps32.u8[0];
		data[15] = temps32.u8[1];
		data[16] = temps32.u8[2];
		data[17] = temps32.u8[3];

	tempflt.flt = caldata.phi;
		data[18] = tempflt.u8[0];
		data[19] = tempflt.u8[1];
		data[20] = tempflt.u8[2];
		data[21] = tempflt.u8[3];

	tempflt.flt = caldata.CalibrationMagnitude;
		data[22] = tempflt.u8[0];
		data[23] = tempflt.u8[1];
		data[24] = tempflt.u8[2];
		data[25] = tempflt.u8[3];

	CompassSend(data,26);
	return(0);

}

#endif //COMPASS_DRIVER_H

