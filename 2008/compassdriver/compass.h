#ifndef COMPASS_H
#define COMPASS_H

/*typedef unsigned char uint_8;
typedef signed long int sint_32;//defining this like this becuase i'm not sure if a long int is 4 bytes or not, so can change later*/
//#include <systypes.h>
/*no errors reporting from anything yet, need to decide big of little endian,
 set in config and driver -- right now assuming little endian*/
/*functions known incomplete: CompassDriver, ~CompassDriver, CompassDriver::spiSend, CompassDriver::spiGet, GetModInfo, GetData, SetDataComponents,
SetConfig, GetConfig, SaveConfig, StartCal, StopCal, need to check endianness of float2bytes and bytes2foat*/

//setcaldata, getcaldata, getdata,setconfig


#include "types.h"



class CompassDriver{
	private://do these need to be static?
	//for future sanity checking	
	int datalength;
	int datapackcount;
	uint_8 lastcommand;
	
	//where the mmap'ed spiregisters are
	volatile unsigned int *ctrlreg0, *ctrlreg1, *datareg, *statusreg, *clockprescalereg, *interuptclearreg;//so member functions can accses the registers
	//spi functions
	int spiinit();
	int spioff();
	spi_status_register spigetstatus();
	int spiSend(uint_8 * data, int size);
	int spiGet(uint_8 * dataresp, int size);
	
	public:
	CompassDriver(int foo);
	~CompassDriver();

	int CompassSend(uint_8 * data, int size);
	//int CompassSend_uint8(uint_8 command);//marked for removal
	ModInfoResp GetModInfo(void);
	compassData GetData(void);
	int SetDataComponents(short int datatypewanted);
	int SetConfig(uint_8 config_id, uint_8 * convigval);//this needs to be able to accept bool, uint_8, and Float32, but i think it will take just bytes
	uint_8 * GetConfig(uint_8 config_id);
	int SaveConfig(void);
	int StartCal(void);
	int StopCal(void);
	CalDataResp GetCalData(void);
	int SetCalData(CalDataResp caldata);
};


//#include "spifunct.h"
#include "spifunct.h.debug"
int CompassDriver::CompassSend(uint_8 * data, int size){

	uint_8 senddata[size+2];
	senddata[0] = sync_flag;
	senddata[size+1] = terminator;
	for(int i=0;i<size; i++){
		senddata[i+1] = data[i];
	}
	CompassDriver::spiSend(senddata, size+2);//send data wrapped with sync_flag and terminator
	return(0);
}

/*int CompassDriver::CompassSend_uint8(uint_8 command){
	uint_8 senddata[3];
	senddata[0] = sync_flag;
	senddata[1] = command;
	senddata[2] = terminator;
	CompassDriver::spiSend(senddata, 3);//send command (single uint_8) wrapped with sync_flag and terminator
}*/

CompassDriver::CompassDriver(int foo){
	//CompassDriver::spiinit();
}

CompassDriver::~CompassDriver(){
	//CompassDriver::spioff();
}
	
ModInfoResp CompassDriver::GetModInfo(void){
	uint_8 data = get_mod_info;
	CompassSend(&data,1);
	uint_8 resp[10];
	CompassDriver::spiGet(resp, 10);
	ModInfoResp reply;
		for(int i = 2;i<6;i++){
			reply.module_type[i-2] = uint_82char2(&resp[i]);//get 2-6 of info into modtype
		}
			reply.module_type[4] = '\0';//appends nul char
		for(int i = 6;i<10;i++){
			reply.firmware_version[i-6] = uint_82char2(&resp[i]);
		}	       
		reply.firmware_version[4] = '\0';
	return(reply);
}

int CompassDriver::SetDataComponents(short int datatypewanted){//input type of data to get, in form of 
	uint_8 data[11];//max size of config + config count + command, excluding header (CompassSend adds it).  the number of bits sent is determind by j, so the unused bits should be ignored
	


	//Generate data stream, and set private "datalength" to length of data to expect
	//datawantedunion.b = {0,1,0,1,0,1,0,1,1}//datawanted should be 65408 for all, to 0 for none
	const uint_8 dataresponsetype[9] = {XRaw, YRaw, XCal, YCal, Heading, Magnitude, Temperature, Distortion, CalStatus};
	const uint_8 dataresponsetypelength[9] = {4, 4, 4, 4, 4, 4, 4, 1, 1};//number of bytes above are
	datapackcount = 0;//reset to zero
	datalength = 0;
	int j = 2;
	for(int i=0;i<10;i++){
		if (checkbitset(datatypewanted,i)){
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


compassData CompassDriver::GetData(void){
	uint_8 data = get_data;
	CompassSend(&data,1);//send command to request data

	uint_8 dataresp[datalength+4];//we will be getting datalength + sync + data_resp + count + term
	CompassDriver::spiGet(dataresp, datalength+4);
		

		compassData datarespstruct;
		//currently ignoring count, use to verify correct num of data packs comming  (ie dataresp[1] == headers in data length (excluding data))
		int i = 3;//first data type packet is byte 4 (3 in array)
	

		if(dataresp[2] == datapackcount){//check for the number of bytes we expect to get
			for(int j = 0;j<datapackcount;j++){
				if(dataresp[i] == XRaw){
					datarespstruct.XRaw = bytes2sint32LE(&dataresp[i+1]);
				i += 5;//go forward 5 bytes to next data header
				}
				if(dataresp[i] == YRaw){
					datarespstruct.YRaw = bytes2sint32LE(&dataresp[i+1]);
					i += 5;
				}
				if(dataresp[i] == XCal){
					datarespstruct.XCal = bytes2floatLE(&dataresp[i+1]);
					i += 5;
				}
				if(dataresp[i] == YCal){
					datarespstruct.YCal = bytes2floatLE(&dataresp[i+1]);
					i += 5;
				}
				if(dataresp[i] == Heading){
					datarespstruct.Heading = bytes2floatLE(&dataresp[i+1]);//degrees			
					i += 5;
				}
				if(dataresp[i] == Magnitude){
					datarespstruct.Magnitude = bytes2floatLE(&dataresp[i+1]);
					i +=5;
				}
				if(dataresp[i] == Temperature){
					datarespstruct.Temperature = bytes2floatLE(&dataresp[i+1]);//Celsius
					i += 5;
				}
				if(dataresp[i] == Distortion){
					datarespstruct.Distortion = dataresp[i+1];//says bool, but guessing send bytes at a time
					i += 2;
				}
				if(dataresp[i] == CalStatus){
					datarespstruct.CalStatus = dataresp[i+1];
					i += 2;
				}
			}
		//else{//we didn't recive the right number of bytes
			//handle the error
		//}
	return(datarespstruct);
	}
}

int CompassDriver::SetConfig(uint_8 config_id, uint_8 * configval){
	uint_8 * temp;
	
	if (config_id == declination){
		uint_8 data[6];
		data[0] = set_config;
		data[1] = config_id;
			data[2] = configval[0];
			data[3] = configval[1];
			data[4] = configval[2];
			data[5] = configval[3];
		CompassSend(data,6);
	}
	else{
		uint_8 data[3];
		data[0] = set_config;
		data[1] = config_id;
		data[3] = configval[0];
		CompassSend(data,3);
	}
return(0);
}

uint_8 * CompassDriver::GetConfig(uint_8 config_id){
	uint_8 data[2];
	data[0] = get_config;
	data[1] = config_id;
	CompassSend(data, 2);

	if (config_id == declination){
		uint_8 resp[8];
		CompassDriver::spiGet(resp, 8);

		uint_8 out[4] = {resp[3],resp[4],resp[5],resp[6]};
		return(out);//returns the second element, assuming it is a byte.
	}
	else{
		uint_8 resp[5];
		CompassDriver::spiGet(resp, 5);
		return(&resp[4]);//returns the second element, assuming it is a byte.
	}
	
}

int CompassDriver::SaveConfig(void){
	uint_8 data = save_config;
	CompassSend(&data,1);
	return(0);
}

int CompassDriver::StartCal(void){
	uint_8 data = start_cal;
	CompassSend(&data,1);
	return(0);//something about wanting to read XRaw and YRaw??? needs to be set before this is run

}

int CompassDriver::StopCal(void){
	uint_8 data = stop_cal;
	CompassSend(&data,1);
	return(0);//need to SaveConfig for this to be permanent
}

CalDataResp CompassDriver::GetCalData(void){
	uint_8 data = get_cal_data;
	CompassSend(&data,1);
	CalDataResp reply;
	uint_8 resp[28];//is this right? needs to be total length of cal data from device.  (4 x sint_32, 2 x float32) = 24 bytes, + count + sync + term + cal_data_resp
	CompassDriver::spiGet(resp, 28);//getting array pf bytes from compass

	//sorting and converting bytes
	//if(resp[2] == 24){//can add this to see of header is correct
	reply.XOffset = bytes2sint32LE(&resp[3]);//needs to pass pointer, so prefix with '&'
	reply.YOffset = bytes2sint32LE(&resp[7]);
	reply.XGain = bytes2sint32LE(&resp[11]);
	reply.YGain = bytes2sint32LE(&resp[15]);
	reply.phi = bytes2floatLE(&resp[19]);
	reply.CalibrationMagnitude = bytes2floatLE(&resp[23]);

	//return structure
	return(reply);
}

int CompassDriver::SetCalData(CalDataResp caldata){
	uint_8 data[26];

	data[0] = set_cal_data;//command
	data[1] = 24;//byte count

	uint_8 * temp;
	temp = sint32_bytesLE(caldata.XOffset);
		data[2] = temp[0];
		data[3] = temp[1];
		data[4] = temp[2];
		data[5] = temp[3];
	
	temp = sint32_bytesLE(caldata.YOffset);
		data[6] = temp[0];
		data[7] = temp[1];
		data[8] = temp[2];
		data[9] = temp[3];

	temp = sint32_bytesLE(caldata.XGain);
		data[10] = temp[0];
		data[11] = temp[1];
		data[12] = temp[2];
		data[13] = temp[3];

	temp = sint32_bytesLE(caldata.YGain);
		data[14] = temp[0];
		data[15] = temp[1];
		data[16] = temp[2];
		data[17] = temp[3];

	temp = float2bytesLE(caldata.phi);
		data[18] = temp[0];
		data[19] = temp[1];
		data[20] = temp[2];
		data[21] = temp[3];

	temp = float2bytesLE(caldata.CalibrationMagnitude);
		data[22] = temp[0];
		data[23] = temp[1];
		data[24] = temp[2];
		data[25] = temp[3];

	CompassSend(data,26);
	return(0);

}

#endif //COMPASS_H

