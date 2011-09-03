#ifndef bottime_h
#define bottime_h



extern long long ROBOT_CORE_START_TIME;		//start time of core in usec
extern char* RUN_TIME_STR;					//time code for this run
long long CORE_TIME();						//returns time of core clock in usec since start of run
void START_CORE_TIME();
void SET_CORE_TIME(long long usec);

/* real time functions*/
char * getTimeStr();
inline unsigned long long nanotime(void);
long long currentTimeMillis();
long long currentTimeMicros();
#endif
