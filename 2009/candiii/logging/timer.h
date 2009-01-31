#ifndef TIMER_H_
#define TIMER_H_

#include <sys/time.h>
#include <sys/resource.h>

#define  ERROR_VALUE -1.0

static int timer_set = FALSE;
static long long old_time;


/* Return the amount of time in useconds used by the current process since it began. */
long long user_time()
{
    struct timeval tv;
    gettimeofday (&tv, (struct timezone*) NULL);
    return ( (tv.tv_sec * 1000000) + (tv.tv_usec) ); // useconds
}


/* Starts timer. */
void start_timer (void)
{
    timer_set = TRUE;
    old_time = user_time();
}


/* Returns elapsed time since last call to start_timer().
   Returns ERROR_VALUE if Start_Timer() has never been called. */
double  elapsed_time (void)
{
    if (timer_set == FALSE)
    {
        return (ERROR_VALUE);
    }
    else
    {
        return 1000000.0 / (user_time() - old_time); // framerate
    }
}


#endif /*TIMER_H_*/


