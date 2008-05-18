#ifndef utilfunc_h
#define utilfunc_h

#include "image.h"
int ImageLoad(char *filename, Image *image);

void LoadGLTextures();
void drawRose(void);
inline unsigned long long nanotime(void);
long long currentTimeMillis();
long long currentTimeMicros();
#define max(a,b) ((a>b)?a:b)
#define min(a,b) ((a<b)?a:b)
char * getTimeStr();
extern long long ROBOT_CORE_START_TIME;		//start time of core in usec
extern char* RUN_TIME_STR;	//time code for this run
long long CORE_TIME();					//returns time of core clock in usec since start of run
void START_CORE_TIME();
void SET_CORE_TIME(long long usec);
#endif
