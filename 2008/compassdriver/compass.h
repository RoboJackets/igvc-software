//no errors reportig from anything yet
//#include <> // spiSend(* data, int size)

#include "spifunct.h"

enum CommandCodes {
	sync_flag = 0xAA,
	terminator = 0x00,
	get_data_ft = 0x04,
	data_resp_ft = 0x05,
	setconfig = 0x06};

enum config_id{
	declination = 0x01,
	true_north = 0x02,
	casamplefreq = 0x03,
	samplefreq = 0x04,
	period = 0x05,
	bigendian = 0x06,
	dampingsize = 0x07};

enum data_cid{
	Xraw = 0x01,
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



class CompassDriver{
	CompassDriver(compassData data);
	~CompassDriver();

	GetModInfo(void);
	GetData(void);
	SetDataComponents();
	SetConfig();
	GetConfig();
	SaveConfig();
	StartCal();
	StopCal();
	GetCalData();
	SetCalData();
	CompassSend();

	short int dataresponsetype;

};



CompassDriver::CompassDriver(){

}

CompassDriver::~CompassDriver(){

}
CompassDriver::GetCalData(){

}

CompassDriver::CompassSend(uint_8 * data, size){

	uint_8 senddata[2+size];
	senddata[0] = sync_flag;
	senddata[size+1] = terminator;
	for(int i=0;i<size; i++){
		senddata[i+1] = data[i];
	}
	spiSend(senddata, size+2);
}

ModInfoResp CompassDriver::GetModInfo(void){
	uint_8 data = get_mod_info;
	compassSend(data, 1);
	uint_8 resp[10];
	spiGet(resp, 10);
	ModInfoResp reply = {resp[1], resp[5]};//syntax???
	return(reply);
}

CompassDriver::SetDataComponents(short int resptype){
	int j = 0;
	for(int i=0; i<0; i++){
		if (dataresptype[i]){
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

CompassDriver::GetData(uint_8 * dataresppoint){
	uint_8 data = get_data_ft;
	CompassSend(data,1);
	//get length of expected response
	for(int i=0; i<0; i++){
		if (dataresptype[i]){
			j++;
		}
	}
	uint_8 dataresp[j+2];
	spiGet(dataresp, j+2);

	dataresppoint = dataresp;//is this the right way??

}

CompassDriver::SetConfig(){

}
CompassDriver::GetConfig(){

}

CompassDriver::SaveConfig(){

}

CompassDriver::StartCal(){

}

CompassDriver::StopCal(){

}

CompassDriver::GetCalData(){

}

CompassDriver::SetCalData(){

}

