/*typedef unsigned char uint_8;
typedef signed long int sint_32;//defining this like this becuase i'm not sure if a long int is 4 bytes or not, so can change later*/
//#include <systypes.h>
//no errors reporting from anything yet, need to decide big of little endian, set in config and driver -- right now assuming little endian
//functions known incomplete: CompassDriver, ~CompassDriver, spiSend, spiGet, GetModInfo, GetData, SetDataComponents, SetConfig, GetConfig, SaveConfig, StartCal, StopCal, GetCalData, SetCalData, float2bytes, bytes2foat




//#include <> // spiSend(* data, int size)

#include "spifunct.h"



enum CommandCodes {
	sync_flag = 0xAA,
	terminator = 0x00,
	get_mod_info = 0x01,
	set_data_components = 0x03,
	get_data = 0x04,
	setconfig = 0x06,
	get_config = 0x07,
	save_config = 0x09,
	start_cal = 0x0A,
	stop_cal = 0x0B,
	get_cal_data = 0x0C,
	set_cal_data = 0x0E};

enum CompassResponse{
	mod_info_resp = 0x02,
	data_resp = 0x05,
	config_resp = 0x08,
	cal_data_resp = 0x0D};

enum Config_Id{
	declination = 0x01,
	true_north = 0x02,
	casamplefreq = 0x03,
	samplefreq = 0x04,
	period = 0x05,
	bigendian = 0x06,
	dampingsize = 0x07};

enum Data_Cid{
	XRaw = 0x01,
	YRaw = 0x02,
	XCal = 0x03,
	YCal = 0x04,
	Heading = 0x05,
	Magnitude = 0x06,
	Temperature = 0x07,
	Distortion = 0x08,
	CalStatus = 0x09};


struct ModInfoResp {
	char module_type[4];
	char firmware_version[4];
};

struct CalDataResp {
	uint_8 ByteCount;
	sint_32 XOffset;
	sint_32 YOffset;
	sint_32 XGain;
	sint_32 YGain;
	float phi;//white paper says Float32, is that what this is?
	float CalibrationMagnitude;//white paper says Float32, is that what this is?
};

struct compassData{
	sint_32 XRaw;
	sint_32 YRaw;
	float XCal;
	float YCal;
	float Heading;
	float Magnitude;
	float Temperature;
	bool Distortion;
	bool CalStatus;
};

//const uint_8 dataresponsetype[9] = {XRaw, YRaw, XCal, YCal, Heading, Magnitude, Temperature, Distortion, CalStatus};

class CompassDriver{
	CompassDriver(compassData data);
	~CompassDriver();

	int CompassSend(uint_8 * data, int size);//send uint8 pointer, and size of array
	int CompassSend_uint8(uint_8 command);//send single uint8
	ModInfoResp GetModInfo(void);
	uint_8 * GetData(uint_8 * dataresppoint);
	int SetDataComponents(bool * resptype);
	int SetConfig(uint_8 config_id, uint_8);//this needs to be able to accept bool, uint_8, and Float32, but i think it will take just bytes
	uint_8 GetConfig(uint_8 config_id);
	int SaveConfig(void);
	int StartCal(void);
	int StopCal(void);
	CalDataResp GetCalData(void);
	int SetCalData(CalDataResp caldata);
	
	//short int dataresponsetype;
	bool * dataresponsetype;

	//bool dataresponsemask[9];//bools in order of dataresponsetype above
	};

int CompassDriver::CompassSend(uint_8 * data, int size){

	uint_8 senddata[2+size];
	senddata[0] = sync_flag;
	senddata[size+1] = terminator;
	for(int i=0;i<size; i++){
		senddata[i+1] = data[i];
	}
	spiSend(senddata, size+2);//send data wrapped with sync_flag and terminator
return(0);
}

int CompassDriver::CompassSend_uint8(uint_8 command){
	uint_8 senddata[3];
	senddata[0] = sync_flag;
	senddata[1] = command;
	senddata[2] = terminator;
	spiSend(senddata, 3);//send command wrapped with sync_flag and terminator
}

CompassDriver::CompassDriver(compassData data){

}

CompassDriver::~CompassDriver(){

}

ModInfoResp CompassDriver::GetModInfo(void){
	uint_8 data = get_mod_info;
	CompassSend_uint8(data);
	uint_8 resp[10];
	spiGet(resp, 10);
	ModInfoResp reply = {resp[1], resp[5]};//syntax???
	return(reply);
}

/*int CompassDriver::SetDataComponents(short int resptype){
	int j = 0;
	for(int i=0; i<0; i++){
		if (dataresponsemask[i]){
			j++;
		}
	}
	uint_8 data[j+1];
	data[0] = set_data_components;
	int k = 1;
	for(int i=1; i<=j; i++){
		if (dataresponsemask[i]){
			data[k++] = (dataresponsetype[k]);
		}
	}
	CompassSend(data, j+1);
	return(0);
}*/


int CompassDriver::SetDataComponents(bool * resptype){
        int j = 0;
        for(int i=0; i<0; i++){
                if (resptype[i]){
                        j++;
                }
        }
        uint_8 data[j];
        int k = 0;
        for(int i=0; i<0; i++){
                if (resptype[i]){
                        data[k++] = (i+1);
                }
        }
        CompassSend(data, j);
        return(0);
}

uint_8 * CompassDriver::GetData(uint_8 * dataresppoint){
	uint_8 data = get_data;
	CompassSend_uint8(data);

	int j =0;
	//get length of expected response
	for(int i=0; i<0; i++){
		if (dataresponsetype[i]){
			j++;
		}
	}
	uint_8 dataresp[j+2];
	spiGet(dataresp, j+2);

	//dataresppoint = dataresp;//is this the right way??
/*	for(int i=1;i<j+1;i++){//this needs to be converted and sorted
		if(dataresp[i] == XRaw){

		if(dataresp[i] == YRaw){
			dataresppoint.YRaw = foo;
		}
		if(dataresp[i] == XCal)
		if(dataresp[i] == YCal
		if(dataresp[i] == Heading)
		if(dataresp[i] == Magnitude)
		if(dataresp[i] == Temperature)
		if(dataresp[i] == Distortion)
		if(dataresp[i] == CalStatus)

	}*/



	return(dataresppoint);//pointer to response including the sync_flag and terminator
}

int CompassDriver::SetConfig(uint_8 config_id, uint_8){
return(0);
}

uint_8 CompassDriver::GetConfig(uint_8 config_id){
	uint_8 data = config_id;
	CompassSend_uint8(config_id);

	uint_8 resp[3];
	spiGet(resp, 3);
	return(resp[2]);//returns the second element, assuming it is a byte.  if the compass returns a bool, it just a byte equal to 1?  the specs show the data field as bytes.
}

int CompassDriver::SaveConfig(void){
	uint_8 data = save_config;
	CompassSend_uint8(data);
	return(0);
}

int CompassDriver::StartCal(void){
	uint_8 data = start_cal;
	CompassSend_uint8(data);
	return(0);//something about wanting to read XRaw and YRaw??? needs to be set before this is run

}

int CompassDriver::StopCal(void){
uint_8 data = stop_cal;
CompassSend_uint8(data);
return(0);//need to SaveConfig for this to be permanent
}

CalDataResp CompassDriver::GetCalData(void){
	uint_8 data = get_cal_data;
	CompassSend_uint8(data);
	CalDataResp reply;
	uint_8 resp[25];//is this right? needs to be total length of cal data from device.  (uint_8, 4 x sint_32, 2 x float32) = 24 bytes, + 1 for head = 25
	spiGet(resp, 25);

	//reply = resp;//need to properly sort the incoming data into a structure (this is wrong).  ignore first byte, then put groups of the next four corrctly

//bool temp[i];

reply.XOffset = bytes2sint32LE(&resp[1]);//needs to pass pointer, so prefix with '&'
reply.YOffset = bytes2sint32LE(&resp[5]);
reply.XGain = bytes2sint32LE(&resp[9]);
reply.YGain = bytes2sint32LE(&resp[13]);

reply.phi;//white paper says Float32, is that what this is?

//(resp[17] + resp[18]*2^8 + ((resp[19] << 1)/2)*2^16) * 2^(((resp[20] >> 7) * 2^7) + ((resp[21] << 1)/2)*2^8)) * ((resp[21] >> 7) * 2^7)//big endian, the mantissa is simply multiplied instead of being 1.mantissa

reply.CalibrationMagnitude;//white paper says Float32, is that what this is?
return(reply);
}

int CompassDriver::SetCalData(CalDataResp caldata){
uint_8 data[25];

data[0] = 24;
//data[1]//convert the rest of the doubles and floats to bytes little endian, send them
spiSend(data,25);
return(0);

}

