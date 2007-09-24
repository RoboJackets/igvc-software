/*typedef unsigned char uint_8;
typedef signed long int sint_32;//defining this like this becuase i'm not sure if a long int is 4 bytes or not, so can change later*/
//#include <systypes.h>
/*no errors reporting from anything yet, need to decide big of little endian,
 set in config and driver -- right now assuming little endian*/
/*functions known incomplete: CompassDriver, ~CompassDriver, spiSend, spiGet, GetModInfo, GetData, SetDataComponents,
SetConfig, GetConfig, SaveConfig, StartCal, StopCal, need to check endianness of float2bytes and bytes2foat*/

//setcaldata, getcaldata, getdata,setconfig


//#include <> // spiSend(* data, int size)

//#include "spifunct.h"



enum CommandCodes {
	sync_flag = 0xAA,
	terminator = 0x00,
	get_mod_info = 0x01,
	set_data_components = 0x03,
	get_data = 0x04,
	set_config = 0x06,
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
	private:
	int datalength;
	int datapackcount;
	public:
	CompassDriver(int foo);
	~CompassDriver();

	int CompassSend(uint_8 * data, int size);//send uint8 pointer, and size of array
	int CompassSend_uint8(uint_8 command);//send single uint8
	ModInfoResp GetModInfo(void);
	compassData GetData(uint_8 * dataresppoint);
	int SetDataComponents(short int datatypewanted);
	int SetConfig(uint_8 config_id, uint_8 * convigval);//this needs to be able to accept bool, uint_8, and Float32, but i think it will take just bytes
	uint_8 GetConfig(uint_8 config_id);
	int SaveConfig(void);
	int StartCal(void);
	int StopCal(void);
	CalDataResp GetCalData(void);
	int SetCalData(CalDataResp caldata);
		
	};

int CompassDriver::CompassSend(uint_8 * data, int size){

	uint_8 senddata[size+2];//this is really two longer
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

CompassDriver::CompassDriver(int foo){
//printf("constructed");
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
       /* union shortint_byte{
		short int si;
		bool b[16];
	} datawantedunion;//crap.  bools are a byte long...

	datawantedunion.si = datatypewanted;*/
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
			datapackcount +=1;//used te check getdata got correct num of packs
			datalength += (dataresponsetypelength[i] + 1);//datalength = length of package + 1 header per package
		}

	}
	data[0] = set_data_components;//command to set data to recive
	data[1] = datapackcount;//count of data we expect
	
        CompassSend(data, j);
        return(0);
}


compassData CompassDriver::GetData(uint_8 * dataresppoint){
	uint_8 data = get_data;
	CompassSend_uint8(data);//send command to request data

	uint_8 dataresp[datalength+4];//we will be getting datalength + sync + data_resp + count + term
	spiGet(dataresp, datalength+4);
		

		compassData datarespstruct;
		//currently ignoring count, use to verify correct num of data packs comming  (ie dataresp[1] == headers in data length (excluding data))
		int i = 3;//first data type packet is byte 4 (3 in array)

		if(dataresp[2] == datapackcount){
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
		//if used as compassData foo = GetData, then members accessed how??
	return(datarespstruct);
	}
else{
//return(-1);//pack count header from wire did not match datapackcount set when config sent.  Maybe retry/resend/query the config?
}

}

int CompassDriver::SetConfig(uint_8 config_id, uint_8 * configval){
	uint_8 * temp;
	
	if (config_id == declination){
		uint_8 data[6];
		data[0] = set_config;
		data[1] = config_id;
		//temp = sint32_bytesLE(configval);
			data[2] = configval[0];
			data[3] = configval[1];
			data[4] = configval[2];
			data[5] = configval[3];
		CompassSend(data,5);
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
	uint_8 resp[28];//is this right? needs to be total length of cal data from device.  (4 x sint_32, 2 x float32) = 24 bytes, + count + sync + term + cal_data_resp
	spiGet(resp, 28);//getting array pf bytes from compass

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

	data[0] = set_cal_data;//set command
	data[1] = 24;//set byte count

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

