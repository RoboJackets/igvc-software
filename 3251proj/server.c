/*///////////////////////////////////////////////////////////
*
* FILE:		server.c
* AUTHOR:	Anthony Gendreau
* PROJECT:	CS 3251 Project 1 - Professor Traynor
* DESCRIPTION:	Network Server Code
*
*////////////////////////////////////////////////////////////

/*Included libraries*/

#include <stdio.h>	  /* for printf() and fprintf() */
#include <sys/socket.h>	  /* for socket(), connect(), send(), and recv() */
#include <arpa/inet.h>	  /* for sockaddr_in and inet_addr() */
#include <stdlib.h>	  /* supports all sorts of functionality */
#include <unistd.h>	  /* for close() */
#include <string.h>	  /* support any string ops */
#include <openssl/sha.h>  /* for SHA1 */

#define RCVBUFSIZE 512		/* The receive buffer size */
#define SNDBUFSIZE 512		/* The send buffer size */
#define BUFSIZE 40		/* Your name can be as many as 40 chars*/
#define SHALENGTH 20		/* SHA1 hash size*/
#define HEXLENGTH 40		/* Formatted Hex string from SHA1 */
#define MAXPENDING 10           /* Maximum number of incoming connections */

//Structure of arguments to pass to client thread
struct ThreadArgs {
	int clntSock; //Socket descriptor for client
}

void handleClient(int clntSock);
int handleCheckId(char *client_id, int sock);
int handleUpdate(char *client_id, char *gps);
int handleFriends(char *client_id, char *friend_list, int sock);
int handleHistory(char *client_id, int sock);
int handleLeave(char *client_id);
int sendData(Message msg, int sock);
char *getGPS(char *id, int type);
int replaceLine(char *id, char *gps);

int main(int argc, char **argv)
{

    int i;				/* Declaring a counter variable*/
    char nameBuf[BUFSIZE];		/* Buff to store name from client */
    unsigned char resultBuf[SHALENGTH]; /* Buff to store change result */
    char answerBuf[HEXLENGTH];		/* Buff contains formatted answer */
    int rcvlen;
    unsigned short servPort = 4000;


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
		int clntSock = accept(serverSocket, (struct sockaddr *) &changeClntAddr, &clntLen);	
	}

	//Create seperate memory for client arguments
	struct ThreadArgs *threadArgs = (struct ThreadArgs *) malloc(sizeof(struct ThreadArgs));
	if(threadArgs == NULL) {
		printf("malloc() failed");
		return 1;
	}
	threadArgs->clntSock =clntSock;

	//Threads creation
	pthread_t threadID;
	int returnValue = pthread_create(&threadID, NULL, ThreadMain, threadArgs);
	if(returnValue != 0) {
		printf("pthread_create() failed with thread %lu\n", (unsigned long int) threadID);
		return 1;
	}

	fclose(file);

	return 0;
}

/*
 * Maintains all the server' connections
 */
void *ThreadMain(void *threadArgs) {
	//Gaurantee thread resources deallocated on return
	pthread_detach(pthread_self());

	//Extra socket file descriptor from argument
	int clntSock = ((struct ThreadArgs *) threadArgs)->clntSock;
	free(threadArgs); //Deallocate memory for argument

	handleClient(clntSock);

	return NULL;
}


//wrap their recieves in while loops and return a message. either call our handle messages directly after decoding what type of message it is 
void handleClient(int clntSock)
{
	char buffer[BUFSIZE];		/* Buff to store name from client */

	ssize_t numBytesRec;
	while(1) {
		/* Receive message from client */
		numBytesRec = recv(clientSock, buffer, BUFSIZE, 0);
		if(numBytesRec < 0) {
			printf("recv() failed");
			break;
		}
	}

	/* Return message to client */
	int totalBytesRec = 0;
	while(numBytesRec > 0) {  //0 indicates end of stream
		//Echo message back to client
		ssize_t numBytesSent = send(clntSock, buffer, numBytesRec, 0);
		if(numBytesSent < 0) {
			printf("send() failed");
			break;
		}
		else if(numBytesSent != numBytesRec) {
			printf("send(), sent unexpected number of bytes");
			break;
		}
		totalBytesRec += numBytesRec;
	}

	//Decode incoming message and call appropriate handler


	close(clntSock);
}

int handleCheckId(char *client_id, int sock)
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
           msg.length = 0;
           msg.data = NULL;
           sendData(msg, sock);
           fclose(file);
           return 0; 
       }
   }
   
   fclose(file);
   free(temp);

   msg.type = MESSAGE_IDAVAILABLE;
   msg.client_id = client_id; 
   msg.length = 0;
   msg.data = NULL;
   
   if(sendData(msg, sock))
   {
       printf("Error Occured during send\n");
       free(msg.client_id);
       return 1;
   }

   free(msg.client_id);
   return 0;
}

int handleUpdate(char *client_id, char *gps)
{
    return replaceLine(client_id, gps);
}

int handleFriends(char *client_id, char *friend_list, int sock)
{
    char *next_friend;
    Message msg;
    char *temp;

    if((msg.client_id = (char *)(malloc(sizeof(char) * strlen(client_id)))) ==
            NULL)
    {
        printf("socket() failed\n");
        exit(1);
    }
    else
        printf("Server Socket: %d\n", serverSock);

    /* Construct local address structure*/
    memset(&changeServAddr, 0, sizeof(changeServAddr));
    changeServAddr.sin_family = AF_INET;
    changeServAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    changeServAddr.sin_port = htons(servPort);

    /* Bind to local address structure */
    if(bind(serverSock, (struct sockaddr *) &changeServAddr, sizeof(changeServAddr)) < 0)
    {
        printf("bind() failed\n");
        exit(1);
    }
    else
        printf("bind() succeeded\n");

    /* Listen for incoming connections */
    if(listen(serverSock, MAXPENDING) < 0)
    {
        printf("listen() failed\n");
        exit(1);
    }
    else
        printf("listen() succeeded\n");

    /* Loop server forever*/
    while(1)
    {

	/* Accept incoming connection */
        clntLen = sizeof(changeClntAddr);
        printf("Client Length: %d\n", clntLen);

        if((clientSock = accept(serverSock, (struct sockaddr *) &changeClntAddr, &clntLen)) < 0)
        {
            printf("accept() failed\n");
            exit(1);
        }
        else
            printf("accept() succeeded\n");

	/* Extract Your Name from the packet, store in nameBuf */
        //while(rcvlen < sizeof(nameBuf))
        //{
            int temp;
            if((temp = recv(clientSock, &nameBuf[rcvlen], sizeof(nameBuf)-rcvlen, 0)) < 0)
            {
                printf("recv() failed\n");
                exit(1);
            }
            else
                printf("recv() returned: %d bytes\n", temp);

            rcvlen += temp;

        //    if(temp == 0)
         //       break;
       //}

	/* Run this and return the final value in answerBuf to client */
	name_changer(nameBuf, resultBuf);
	for (i = 0; i < SHALENGTH; i++) 
	{
	  sprintf(&answerBuf[i*2],"%02x ", resultBuf[i]);
	}
	printf("%s\n", answerBuf);


	/* Return answerBuf to client */
        if(send(clientSock, answerBuf, sizeof(answerBuf), 0) != sizeof(answerBuf))
        {
            printf("send() failed\n");
            exit(1);
        }
        else
            printf("send() succeeded\n");

        if((close(clientSock)) < 0)
        {
            printf("close() failed\n");
            exit(1);
        }
    }

    return 0;
}

/* Takes the client name and changes it */
/* Students should NOT touch this code */
void name_changer(char *nameBuf, char *resultBuf) 
{
    SHA1((unsigned char *)nameBuf, strlen(nameBuf), (unsigned char *)resultBuf);
}


