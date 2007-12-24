#ifndef COMPASS_TYPES_H
#define COMPASS_TYPES_H

typedef unsigned char uint_8;
typedef signed int sint_32;

//default configuration
#define DEFAULT_DECLINATION 0
#define DEFAULT_TRUE_NORTH 1
#define DEFAULT_CALSAMPLE_FREQ 8
#define DEFAULT_SAMPLE_FREQ 0
#define DEFAULT_PERIOD 5
#define DEFAULT_BIG_ENDIAN 0
#define DEFAULT_DAMPING_SIZE  1

//defualt types
#define DEFAULT_XRAW 0
#define DEFAULT_YRAW 0
#define DEFAULT_XCAL 0
#define DEFAULT_YCAL 0
#define DEFAULT_HEADING 1
#define DEFAULT_MAGNITUDE 1
#define DEFAULT_TEMPERATURE 0
#define DEFAULT_DISTORTION 1
#define DEFAULT_CALSTATUS 1

enum CommandCodes {
	sync_flag = 0xAA,
	terminator = 0x00,
	get_mod_info = 0x01,
	set_data_components = 0x03,
	get_data= 0x04,
	set_config= 0x06,
	get_config= 0x07,
	save_config= 0x09,
	start_cal = 0x0A,
	stop_cal = 0x0B,
	get_cal_data = 0x0C,
	set_cal_data = 0x0E
};

enum CompassResponse {
	mod_info_resp = 0x02,
	data_resp = 0x05,
	config_resp = 0x08,
	cal_data_resp = 0x0D
};

enum Config_Id {
	declination = 0x01,
	true_north = 0x02,
	calsamplefreq = 0x03,
	samplefreq = 0x04,
	period = 0x05,
	bigendian = 0x06,
	dampingsize = 0x07
};

enum Data_Cid {
	XRaw = 0x01,
	YRaw = 0x02,
	XCal = 0x03,
	YCal = 0x04,
	Heading = 0x05,
	Magnitude = 0x06,
	Temperature = 0x07,
	Distortion = 0x08,
	CalStatus = 0x09
};

typedef union s32_u8{
		sint_32 s32;
		uint_8 u8[4];
	};

typedef	union float_u8{
		float flt;
		uint_8 u8[4];
	};

typedef union char_u8{
		uint_8 u8;
		char chr;
	};

struct ModInfoResp {
	char module_type[5];//four chars read from compass + null char i add later
	char firmware_version[5];
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

struct compassData {//all posible types the compass can sends. when asked Distortion and Calstatus are suposed to be bools, but i think they are read a byte at a time, so should work as uint_8's.
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

const uint_8 dataresponsetype[9] = {XRaw, YRaw, XCal, YCal, Heading, Magnitude, Temperature, Distortion, CalStatus};//used so for loop can scroll through union
const uint_8 dataresponsetypelength[9] = {4, 4, 4, 4, 4, 4, 4, 1, 1};//number of bytes data is

union DataRespType {
	struct {//is there a way to acces this like an array, ie datarespvar.bits[2]?
		short int XRaw:1;
		short int YRaw:1;
		short int XCal:1;
		short int YCal:1;
		short int Heading:1;
		short int Magnitude:1;
		short int Temperature:1;
		short int Distortion:1;
		short int CalStatus:1;
		short int :7;
	};
	short int sint;

};
struct ConfigData {
	float declination;
	uint_8 truenorth;//supp to be bool
	uint_8 calsamplefreq;
	uint_8 samplefreq;
	uint_8 period;
	uint_8 bigendian;//supp to be bool
	uint_8 dampingsize;
};

#endif //COMPASS_TYPES_H
