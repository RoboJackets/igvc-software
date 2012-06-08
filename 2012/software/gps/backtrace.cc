#include <stdio.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <execinfo.h>
#include <unistd.h>


static char exeprtthingy____[32768];
char * getexe() {
	char link[32768];
    
    snprintf(link,sizeof link,"/proc/%d/exe",getpid());
    if(readlink(link,exeprtthingy____,sizeof exeprtthingy____)==-1) {
        fprintf(stderr,"ERROR getting exe name\n");
    }
    return exeprtthingy____;
}

void stacktrace() {

  void *trace[16];
  char **messages = (char **)NULL;
  int i, trace_size = 0;

  trace_size = backtrace(trace, 16);

  messages = backtrace_symbols(trace, trace_size);
  /* skip first stack frame (points here) */
  printf("[bt] Execution path:\n");
  for (i=1; i<trace_size; ++i)
  {
    //printf("[bt] #%d %s\n", i, messages[i]);

    char syscom[256];
    sprintf(syscom,"addr2line %p -e %s", trace[i],getexe()); //last parameter is the name of this app
    system(syscom);
  }


}


    




