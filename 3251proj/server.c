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
#define PINGTIMEOUT   200

int count = 0;

void handleClient(int serverSocket);
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
        int pingTimer = 0;              //Time since last ping reset with every
                                        //ping

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
        printf("Unable to malloc space for the message\n");
        return 1;
    }

    if((msg.data = (char *)(malloc(sizeof(char) * MAXNUMREQUESTS * (MAXIDLEN +
                            20)))) == NULL)
    {
        printf("Unable to malloc space for the message\n");
        free(msg.client_id);
        return 1;
    }

    if((temp = (char *)(malloc(sizeof(char) * (MAXIDLEN + 20)))) == NULL)
    {
        printf("Unable to malloc space\n");
        free(msg.client_id);
        free(msg.data);
        return 1;
    }

    msg.client_id = client_id;

    next_friend = strtok(friend_list, "\n");
    while(next_friend != NULL)
    { 
        temp = getGPS(next_friend, MESSAGE_FRIENDS);
        strcat(msg.data, temp);
        next_friend = strtok(NULL, "\n");
    }

    msg.length = strlen(msg.data);

    if(sendData(msg, sock))
    {
        printf("Error Occured during Send\n");
        return 1;
    }

    return 0;
}

int handleHistory(char *client_id, int sock)
{
    Message msg;

    if((msg.client_id = (char *)(malloc(sizeof(char) * strlen(client_id)))) ==
            NULL)
    {
        printf("Unable to malloc space for the message\n");
        return 1;
    }

    if((msg.data = (char *)(malloc(sizeof(char) * MAXLINELENGTH))) ==
            NULL)
    {
        printf("Unable to malloc space for the message\n");
        free(msg.client_id);
        return 1;
    }
    
    msg.type = MESSAGE_HISTORY;
    strcpy(msg.client_id, client_id);
    msg.data = getGPS(client_id, MESSAGE_HISTORY);
    msg.length = strlen(msg.data);

    if(sendData(msg, sock))
    {
        printf("Error Occured during sendData\n");
        free(msg.client_id);
        free(msg.data);
        return 1;
    }

    free(msg.client_id);
    free(msg.data);
    return 0;
}

/* This function simply removes the client from the file it does not 
 * close the connection that has to be done in handleClient */
int handleLeave(char *client_id)
{
    return replaceLine(client_id, NULL);
}

char *getGPS(char *id, int type)
{
    char *line, *lat, *lon;
    FILE *fp;

    char *temp;
    fp = fopen("data.txt", "r");

    if((line = (char *)(malloc(sizeof(char) * (MAXIDLEN + 20)))) == NULL)
    {
        printf("Unable to malloc space\n");
        return NULL;
    }

    while(fgets(line, MAXIDLEN, fp))
    {
        temp = strtok(line, " "); //get the first token

        if(strcmp(temp, id) == 0)
        {
            if(type == MESSAGE_FRIENDS)
            {
                lat = strtok(NULL, " "); //get the latitude
                lon = strtok(NULL, " "); //get the longitude
                sprintf(line, "%s: %s %s", temp, lat, lon);
            }
            else if(type == MESSAGE_HISTORY)
            {
               //Do Nothing I want the whole line         
            }
            return line;
        }
    }

    sprintf(line, "%s: Not Found\n", id);
    return line;
}

int replaceLine(char *client_id, char *gps)
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

           if(gps != NULL) //make the new line
           {
               sprintf(temp, "%s %s %s", client_id, gps, temp);
           }
           else //this was a leave clear the line
           {
               sprintf(temp, "");
           }
           found = 1;          
       }
       fputs(temp, out); //Write the line (edited or not)
   }
   
   if(!found && gps != NULL)
   {
       sprintf(temp, "%s %s", client_id, gps);
       fputs(temp, out); //Write the line
   }

   fclose(in);
   fclose(out);

   //Replace the old file with the new one
   remove("data.txt");
   rename("temp.txt", "data.txt");

   return 0;
}

/*
 * Sends the data (serialized) to the client 
 */
int sendData(Message msg, int sock)
{
    ssize_t numBytes = 0;
    int bufLen;

    //Send the msg
    bufLen = strlen((char *)(&msg));
    numBytes = send(sock, (char *)(&msg), bufLen, 0);
    if(numBytes < 0)
    {
        printf("send() Failed\n");
        return 1;
    }
    else if(numBytes != bufLen)
    {
        printf("send() sent the wrong number of bytes\n");
        return 1;
    }

    return 0;
}
