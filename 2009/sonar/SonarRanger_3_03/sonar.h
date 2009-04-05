

#define CMD_READALL 60
#define CMD_PINGALL 62
#define CMD_PINGINDIV 112
#define CMD_READINDIV 114
#define CMD_AUTOPILOT 94
#define CMD_SETFREQ 37
#define CMD_SETALLGAIN 38
#define CMD_SETALLMRANGE 124
#define CMD_WARNMODE 33
#define CMD_SETINDIVGAIN 43
#define CMD_SETINDIVMRANGE 45
#define CMD_SETALLWARNTHRESH 61
#define CMD_SETINDIVWARNTHRESH 95
#define CMD_SETTOTALSTEPS 35
#define CMD_SETINDIVSTEP 36
#define CMD_SETDEVICEACTIVE 64
#define CMD_SETECHONUM 126
#define CMD_DEFAULTSETTINGS 68
#define CMD_GET 58
#define CMD_GETINDIVGAIN 43
#define CMD_GETINDIVMRANGE 45
#define CMD_GETINDIVWARNTHRESH 61
#define CMD_GETINDIVACTIVE 64
#define CMD_GETINDIVSTEPID 36
#define CMD_GETFREQ 37
#define CMD_GETSTEPTOTAL 35
#define CMD_ON 49
#define CMD_OFF 48
#define DBG_C 67
#define DBG_J 74
#define DBG_M 77
#define DFLT_STEPFREQ 14
#define DFLT_TOTALSTEPS 10
#define DFLT_WARNTHRESH 660
#define DFLT_MAXDIST 4000
#define DFLT_GAIN 1
#define DFLT_ADDRSTART 112
#define DFLT_ECHONUM 1
#define FLAG_ACTIVE_SET		0x01
#define FLAG_CONT_SET		0x02
#define FLAG_WARN_SET		0x04
#define FLAG_READ_SET		0x08
#define	FLAG_PING_SET		0x10
#define FLAG_GEDIT_SET		0x20
#define FLAG_MREDIT_SET		0x40

typedef unsigned char byte;

typedef struct _sondev{
	byte addr;
	byte gain;
	byte mrange;
	byte wthresh;
	byte seqid;
	byte rdata[2];
	byte flags; //mrangechange_flag,gainchange_flag,ping_flag, read_flag, warn_flag, cont_flag, active_bit
}SONDEV;





//============FXNS=====================
void SonDev_Start(int num, int initGain, int initMRange, int freq);
void SonDev_Run(void (*waitfxn)(long unsigned int)); //for some reason, a direct call to delay freezes arduino
byte* SonDev_Process(byte* data);
void StartRange(int Addr);
void GetRange(int Addr, int EchoNum, byte* rdat);
void SetGain(byte Addr, byte GainVal);
void SetMaxRange(byte Addr, byte RangeVal);
