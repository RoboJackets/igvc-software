#include <stdio.h>	  /* for printf() and fprintf() */
#include <sys/socket.h>	  /* for socket(), connect(), send(), and recv() */
#include <arpa/inet.h>	  /* for sockaddr_in and inet_addr() */
#include <stdlib.h>	  /* supports all sorts of functionality */
#include <unistd.h>	  /* for close() */
#include <string.h>	  /* support any string ops */
#include <pthread.h>
#include "server.h"
#include "typedef.h"

int main(int argc, char **argv)
{
   //TODO: Set up server
   
   while(1)
   {
       runServer();
   }
}

/*
 * Maintains all the server's connections and handles all server operations 
 */
void runServer()
{

}
