#define DFLT_STEPFREQ 14
#define DFLT_TOTALSTEPS 10
#define DFLT_WARNTHRESH 660
#define DFLT_MAXDIST 4000
#define DFLT_GAIN 1
#define DFLT_ADDRSTART 112
#define DFLT_ECHONUM 1
#define FLAG_ACTIVE_SET		0x01
#define FLAG_CONT_SET		0x02
#define FLAG_READ_SET		0x04
#define	FLAG_PING_SET		0x08
#define FLAG_GEDIT_SET		0x10
#define FLAG_MREDIT_SET		0x20
#undef int

typedef unsigned char byte;

typedef struct _sondev{
	byte addr;
	byte gain;
	byte mrange;
	byte seqid;
	byte rdata[2];
	byte flags; //mrangechange_flag,gainchange_flag,ping_flag, read_flag, warn_flag, cont_flag, active_bit
}SONDEV;

enum sd_optype_t{SREAD_ALL,SPING_ALL,SREAD_IND,SPING_IND,SET_GAIN_ALL,SET_GAIN_IND,SET_MRANGE_ALL,SET_MRANGE_IND,
			SET_FREQ,SET_STEP_IND,SET_STEP_TOT,SET_ACTIVE_IND,SET_ECHONUM,GET_GAIN_IND,GET_MRANGE_IND,
			GET_ACTIVE_IND,GET_STEP_IND,GET_FREQ,GET_STEP_TOT,START_SONDEV, SET_AUTO_ALL, SDEV_ADDRSET};

//============FXNS=====================
void SonDev_Start(int num, int initGain, int initMRange, int freq);
void SonDev_Run(long unsigned int (*millis_fxn)());
void SonDev_Process(byte* data, void (*sendpktfxn)(char cmd, int dataSize, byte* data));
void StartRange(int Addr);
void GetRange(int Addr, int EchoNum, byte* rdat);
void SetGain(byte Addr, byte GainVal);
void SetMaxRange(byte Addr, byte RangeVal);
//======================================

