/*typedef unsigned char uint_8;
typedef signed long int sint_32;//defining this like this becuase i'm not sure if a long int is 4 bytes or not, so can change later*/
//#include <systypes.h>
/*no errors reporting from anything yet, need to decide big of little endian,
 set in config and driver -- right now assuming little endian*/
/*functions known incomplete: CompassDriver, ~CompassDriver, spiSend, spiGet, GetModInfo, GetData, SetDataComponents,
SetConfig, GetConfig, SaveConfig, StartCal, StopCal, GetCalData, SetCalData,  need to check endianness of float2bytes and
bytes2foat*/




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
	float phi;
	float CalibrationMagnitude;
};

struct compassData{//all posible values of data compass sends when asked Distortion and Calstatus are said to be bools, but i think they are read a byte at a time, so should work as uint_8's.
	sint_32 XRaw;
	sint_32 YRaw;
	float XCal;
	float YCal;
	float Heading;
	float Magnitude;
	float Temperature;
	uint_8 Distortion;
	uint_8 CalStatus;
};

//const uint_8 dataresponsetype[9] = {XRaw, YRaw, XCal, YCal, Heading, Magnitude, Temperature, Distortion, CalStatus};

class CompassDriver{
	Private:
	int datalength;
	Public:
	CompassDriver(compassData data);
	~CompassDriver();

	int CompassSend(uint_8 * data, int size);//send uint8 pointer, and size of array
	int CompassSend_uint8(uint_8 command);//send single uint8
	ModInfoResp GetModInfo(void);
	compassData GetData(uint_8 * dataresppoint);
	int SetDataComponents(short int datatypewanted);
	int SetConfig(uint_8 config_id, uint_8);//this needs to be able to accept bool, uint_8, and Float32, but i think it will take just bytes
	uint_8 GetConfig(uint_8 config_id);
	int SaveConfig(void);
	int StartCal(void);
	int StopCal(void);
	CalDataResp GetCalData(void);
	int SetCalData(CalDataResp caldata);
		
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
	spiSend(senddata, 3);//send command (single uint_8) wrapped with sync_flag and terminator
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

int CompassDriver::SetDataComponents(short int datatypewanted){//input type of data to get, in form of 
        union shortint_byte{
		short int si;
		bool b[16];
	} datawantedunion;

	datawantedunion.si = datatypewanted;
	uint_8 data[11];//max size of config + config count + command, excluding header (compassSend adds it).  the number of bits sent is determind by j, so the unused bits should be ignored
	


	//Generate data stream, and set private "datalength" to length of data to expect
	//datawantedunion.b = {0,1,0,1,0,1,0,1,1}//datawanted should be 65408 for all, to 0 for none
	const uint_8 dataresponsetype[9] = {XRaw, YRaw, XCal, YCal, Heading, Magnitude, Temperature, Distortion, CalStatus};
	const uint_8 dataresponsetypelength[9] = {4, 4, 4, 4, 4, 4, 4, 1, 1};//number of bytes above are
	int j = 2;
	for(int i=0;i<10;i++){
		if (datawantedunion.b[i]){
			data[j] = dataresponsetype[i];
			j++;
			datalength += (dataresponsetypelength[i] + 1);//datalength = length of package + 1 header per package
		}
	}
	data[0] = set_data_components;//command to set data to recive
	data[1] = j-1;//count of data we expect
	
        CompassSend(data, j);
        return(0);
}


compassData CompassDriver::GetData(uint_8 * dataresppoint){
	uint_8 data = get_data;
	CompassSend_uint8(data);//send command to request data

	datalength

	uint_8 dataresp[datalength+3];//we will be getting datalength + count + sync + term
	spiGet(dataresp, datalength+3);
		

		compassData datarespstruct;
		//currently ignoring count, use to verify correct num of data packs comming  (ie dataresp[1] == headers in data length (excluding data))
		int i = 2;
		if(dataresp[i] == XRaw){
			datarespstruct.XRaw = byte2sint32(&dataresp[i]);
		i += 5;//go forward 5 bytes to next data header
		}
		if(dataresp[i] == YRaw){
			datarespstruct.YRaw = byte2sint32(&dataresp[i]);
		i += 5;
		}
		if(dataresp[i] == XCal){
			datarespstruct.XCal = byte2float(&dataresp[i]);
		i += 5;
		}
		if(dataresp[i] == YCal){
			datarespstruct.YCal = byte2float(&dataresp[i]);
			i += 5;
		}
		if(dataresp[i] == Heading){
			datarespstruct.Heading = byte2float(&dataresp[i]);//degrees			
			i += 5;
		}
		if(dataresp[i] == Magnitude){
			datarespstruct.Magnitude = byte2float(&dataresp[i]);
			i +=5;
		}
		if(dataresp[i] == Temperature){
			datarespstruct.Temperature = byte2float(&dataresp[i]);//Celsius
			i += 5;
		}
		if(dataresp[i] == Distortion){
			datarespstruct.Distortion = dataresp[i];//says bool, but guessing send bytes at a time
			i += 1;
		}
		if(dataresp[i] == CalStatus){
			datarespstruct.CalStatus = dataresp[i];
			i += 1;
		}
		//if used as compassData foo = GetData, then members accessed how??
	return(datarespstruct);
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
	CalDataResp reply;//define reply as structure of cal data resp
	uint_8 resp[25];//is this right? needs to be total length of cal data from device.  (uint_8, 4 x sint_32, 2 x float32) = 24 bytes, + 1 for head = 25
	spiGet(resp, 25);//getting array pf bytes from compass

//sorting and converting bytes
reply.XOffset = bytes2sint32LE(&resp[1]);//needs to pass pointer, so prefix with '&'
reply.YOffset = bytes2sint32LE(&resp[5]);
reply.XGain = bytes2sint32LE(&resp[9]);
reply.YGain = bytes2sint32LE(&resp[13]);
reply.phi = bytes2floatLE(&resp[17]);
reply.CalibrationMagnitude = bytes2floatLE(&resp[21]);

//return structure
return(reply);
}

int CompassDriver::SetCalData(CalDataResp caldata){
uint_8 data[25];

data[0] = 24;
//data[1]//convert the rest of the doubles and floats to bytes little endian, send them
spiSend(data,25);
return(0);

}

