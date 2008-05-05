//this is from compassTypes
/*
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
*/

//the can be hardcoded from the #defines above

ConfigData defaultconfig;

defaultconfig.declination = DEFAULT_DECLINATION;
defaultconfig.truenorth = DEFAULT_TRUE_NORTH;//supp to be bool
defaultconfig.calsamplefreq = DEFAULT_CALSAMPLE_FREQ;
defaultconfig.samplefreq = DEFAULT_SAMPLE_FREQ;
defaultconfig.period = DEFAULT_PERIOD;
defaultconfig.bigendian = DEFAULT_BIG_ENDIAN;//supp to be bool
defaultconfig.dampingsize = DEFAULT_DAMPING_SIZE;


DataTypeReq defaultdatatype;

defaultdatatype.xraw = DEFAULT_XRAW;
defaultdatatype.yraw = DEFAULT_YRAW;
defaultdatatype.xcal = DEFAULT_XCAL;
defaultdatatype.ycal = DEFAULT_YCAL;
defaultdatatype.heading = DEFAULT_HEADING;
defaultdatatype.magnitude = DEFAULT_MAGNITUDE;
defaultdatatype.temperature = DEFAULT_TEMPERATURE;
defaultdatatype.distortion = DEFAULT_DISTORTION;
defaultdatatype.calstatus = DEFAULT_CALSTATUS;

