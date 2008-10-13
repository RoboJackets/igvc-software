#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include "bottime.h"
#include <stdio.h> 

long long ROBOT_CORE_START_TIME;		//start time of core in usec
char* RUN_TIME_STR=(char *)"00.00.00.00.00.00";	//time code for this run







char * getTimeStr(){
	time_t tsec;
	struct tm t;
	char *c;
	c=(char *)malloc(19*sizeof(char));
	tsec=time((time_t *)0);
	localtime_r(&tsec,&t);
	sprintf(c,"%.2d.%.2d.%.2d.%.2d.%.2d.%.2d",t.tm_year-100,t.tm_mon+1,t.tm_mday,t.tm_hour,t.tm_min,t.tm_sec);
	return c;
}

long long CORE_TIME(){
   return currentTimeMicros()-ROBOT_CORE_START_TIME;
}

void START_CORE_TIME(){
	ROBOT_CORE_START_TIME=currentTimeMicros();
	RUN_TIME_STR=getTimeStr();
}
void SET_CORE_TIME(long long usec){
	ROBOT_CORE_START_TIME=currentTimeMicros()-usec;
}





inline unsigned long long nanotime(void){
#ifdef __i386__
     unsigned long long val;
     //asm( "statements" : output_registers : input_registers : clobbered_registers);
     //or _asm_ )"statements" : output_registers : input_registers : clobbered_registers);
    __asm__ __volatile__("rdtsc" : "=A" (val) : );
     return(val);
#else
	// This is architecture-independent but provides low granularity
	return 1000*currentTimeMicros();
#endif
}


long long currentTimeMillis() {
   long long t;
   struct timeval tv;

   gettimeofday(&tv, (struct timezone*)NULL);

   t = tv.tv_sec;
   t = (t * 1000) + (tv.tv_usec/1000);

   return t;
}

long long currentTimeMicros() {
   long long t;
   struct timeval tv;

   gettimeofday(&tv, (struct timezone*)NULL);

   t = tv.tv_sec;
   t = (t * 1000 * 1000) + (tv.tv_usec);

   return t;
}
