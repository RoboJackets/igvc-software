#include <stdio.h>	  /* for printf() and fprintf() */
#include <sys/socket.h>	  /* for socket(), connect(), send(), and recv() */
#include <arpa/inet.h>	  /* for sockaddr_in and inet_addr() */
#include <stdlib.h>	  /* supports all sorts of functionality */
#include <unistd.h>	  /* for close() */
#include <string.h>	  /* support any string ops */
#include <pthread.h>
#include "server.h"
#include "typedef.h"
#include <pthread.h>

/* These may not be used, 2/21 */
#define RCVBUFSIZE 512		/* The receive buffer size */
#define SNDBUFSIZE 512		/* The send buffer size */
#define BUFSIZE 40		/* Your name can be as many as 40 chars*/
#define SHALENGTH 20		/* SHA1 hash size*/
#define HEXLENGTH 40		/* Formatted Hex string from SHA1 */
#define MAXPENDING 50 /* Maximum number of incoming connections */
#define MAXLINELENGTH 5000


int count = 0;

void handleClient(int serverSocket);
int handleConnect(char *client_id);
int handleUpdate(char *client_id, char *gps);
int handleFriends(char *client_id, char *friend_list);
int handleHistory(char *client_id);
int handlePing(char *client_id);
int handleLeave(char *client_id);
int sendData(Message msg);

int main(int argc, char **argv)
{
	//File setup
	FILE *file;
	file = fopen("data.txt","a+"); //fprintf(file,"%s","To write");

	//Threads initialization
//	int count = 0;
	int maxAllowed = MAXPENDING;
	pthread_t *client_id = (pthread_t*) malloc(sizeof(pthread_t)*maxAllowed);
	if(client_id == NULL) exit (1);

	//TODO: Set up server
	int serverSock;			/* Server Socket */
	struct sockaddr_in changeServAddr;	/* Local address */
	unsigned short changeServPort;	/* Server port */


	/* Create new TCP Socket for incoming requests*/
	if((serverSock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0) {
		printf("socket() failed");
	}

  /* Construct local address structure*/
  memset(&changeServAddr, 0, sizeof(changeServAddr));
	changeServAddr.sin_family = AF_INET;
	changeServAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	changeServAddr.sin_port = htons(2567); /* changeServPort*/
    
  /* Bind to local address structure */
	if(bind(serverSock, (struct sockaddr *) &changeServAddr, sizeof(changeServAddr)) < 0) {
		printf("bind() failed");
	}

  /* Listen for incoming connections */
  if(listen(serverSock, MAXPENDING) < 0) {
		printf("listen() failed");
	}



	while(1)
	{
		//pthread_create(client_id,NULL,handleClient(serverSock),&count);
		count++;
	}

	fclose(file);

	return 0;
}

/*
 * Maintains all the server' connections
 */
void handleClient(int serverSocket)
{
	int clientSock;			/* Client Socket */
	int i;				/* Declaring a counter variable*/
	struct sockaddr_in changeClntAddr;	/* Client address */
	char nameBuf[BUFSIZE];		/* Buff to store name from client */
	unsigned char resultBuf[SHALENGTH]; /* Buff to store change result */
	char answerBuf[HEXLENGTH];		/* Buff contains formatted answer */
	unsigned int clntLen;		/* Length of address data struct */

	/* Accept incoming connection */
	clntLen = sizeof(changeClntAddr);
	if((clientSock = accept(serverSocket, (struct sockaddr *) &changeClntAddr, &clntLen)) < 0) {
		printf("accept() failed");
	}
	
	/* printf("HIT\n"); */
	/* Extract Your Name from the packet, store in nameBuf */
	if((recv(clientSock, nameBuf, BUFSIZE, 0)) < 0) {
		printf("recv() failed");
	}

	/* Run this and return the final value in answerBuf to client */
	for (i = 0; i < SHALENGTH; i++) 
	{
  	sprintf(&answerBuf[i*2],"%02x ", resultBuf[i]);
	}
	printf("%s\n", answerBuf);

	/* Return answerBuf to client */
	if(send(clientSock, answerBuf, HEXLENGTH, 0) != HEXLENGTH) {
		printf("send() failed");
	}

close(clientSock);
pthread_exit(0);
count--;
}

int handleConnect(char *client_id)
{
   Message msg;
   char *temp;

   //Setup the file
   FILE *file;
   file = fopen("data.txt","r"); //fprintf(file,"%s","To write");
   
   if((msg.client_id = (char *)(malloc(sizeof(char) * strlen(client_id)))) == NULL)
   {
       printf("Unable to malloc space for the client's id\n");
       return -1;
   }
   
   if((temp = (char *)(malloc(sizeof(char) * strlen(client_id)))) == NULL)
   {
       printf("Unable to malloc space for the client's id\n");
       return -1;
   }


   while(fgets(temp, strlen(temp), file) != NULL)
   {
       if(strcmp(temp, client_id) == 0) //Found that Id already
       {
           msg.type = MESSAGE_IDTAKEN;
           msg.client_id = client_id; 
           sendData(msg);
           fclose(file);
           return 0; 
       }
   }

   msg.type = MESSAGE_IDAVAILABLE;
   msg.client_id = client_id; 
   sendData(msg);

   fclose(file);
   free(msg.client_id);
   free(temp);
   return 0;
}

int handleUpdate(char *client_id, char *gps)
{
   //Setup the files
   FILE *in;
   in = fopen("data.txt","r"); //fprintf(file,"%s","To write");
   FILE *out;
   out = fopen("temp.txt", "w");

   char *temp;
   char *id;
   int found = 0;

   if((temp = (char *)(malloc(sizeof(char) * strlen(client_id)))) == NULL)
   {
       printf("Unable to malloc space for the file lines\n");
       return 1;
   }

   while(fscanf(in, "%s\n", temp) != EOF)
   {
       id = strtok(temp, " "); //get the first token as the id

       if(strcmp(id, client_id) == 0) //Found that Id already
       {
           temp = strtok(temp, " "); //Remove the user id on temp
           sprintf(temp, "%s %s %s", client_id, gps, temp);
           found = 1;          
       }
       fputs(temp, out); //Write the line (edited or not)
   }
   
   if(!found)
   {
       sprintf(temp, "%s %s", client_id, gps);
       fputs(temp, out); //Write the line (edited or not)
   }

   fclose(in);
   fclose(out);

   //Replace the old file with the new one
   remove("data.txt");
   rename("temp.txt", "data.txt");

   return 0;
}

int handleFriends(char *client_id, char *friend_list)
{
    return 0;
}

int handleHistory(char *client_id)
{
    return 0;
}

int handlePing(char *client_id)
{
    return 0;
}

int handleLeave(char *client_id)
{
    return 0;
}


int sendData(Message msg)
{
    return 0;
}
