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

//Structure of arguments to pass to client thread
struct ThreadArgs {
	int clntSock; //Socket descriptor for client
};

pthread_mutex_t file_lock = PTHREAD_MUTEX_INITIALIZER;

void handleClient(int clntSock);
int handleCheckId(char *client_id, int sock);
int handleUpdate(char *client_id, char *gps);
int handleFriends(char *client_id, char *friend_list, int sock);
int handleHistory(char *client_id, int sock);
int handleLeave(char *client_id);
int handlePing(char *client_id, int sock);
int sendData(Message msg, int sock);
char *getGPS(char *id, int type);
int replaceLine(char *id, char *gps);
void *ThreadMain(void *threadArgs);
int executeSend(char *buf, int sock);
char *executeReceive(int sock, int size);
Message receiveData(int sock);
char *readLine(FILE *fp);
char *getFriend(char *friend_list, int index);

int main(int argc, char **argv)
{
	//TODO: Set up server
	int serverSock;			/* Server Socket */
	struct sockaddr_in changeServAddr;	/* Local address */
	unsigned short changeServPort = 4000;	/* Server port */
        struct sockaddr_in changeClntAddr;
        socklen_t clntLen = sizeof(changeClntAddr);


	/* Create new TCP Socket for incoming requests*/
	if((serverSock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0) {
		printf("socket() failed\n");
                return 1;
	}
        else
            printf("Server Socket: %d\n", serverSock);


  /* Construct local address structure*/
  memset(&changeServAddr, 0, sizeof(changeServAddr));
	changeServAddr.sin_family = AF_INET;
	changeServAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	changeServAddr.sin_port = htons(changeServPort); /* changeServPort*/
    
  /* Bind to local address structure */
	if(bind(serverSock, (struct sockaddr *) &changeServAddr, sizeof(changeServAddr)) < 0) {
		printf("bind() failed\n");
                return 1;
	}
        else
            printf("bind() succeeded\n");


  /* Listen for incoming connections */
  if(listen(serverSock, MAXPENDING) < 0) {
		printf("listen() failed\n");
                return 1;
	}
  else
      printf("listen() succeeded\n");

	while(1)
	{
		int clntSock = accept(serverSock, (struct sockaddr *) &changeClntAddr, &clntLen);	
	

	        //Create seperate memory for client arguments
        	struct ThreadArgs *threadArgs = (struct ThreadArgs *) malloc(sizeof(struct ThreadArgs));
	        if(threadArgs == NULL) {
	        	printf("malloc() failed");
        		return 1;
        	}
        	threadArgs->clntSock = clntSock;

        	//Threads creation
        	pthread_t threadID;
        	int returnValue = pthread_create(&threadID, NULL, ThreadMain, threadArgs);
        	if(returnValue != 0) {
        		printf("pthread_create() failed with thread %lu\n", (unsigned long int) threadID);
        		return 1;
        	}

        }
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
        int leave = 0;
        Message msg;
        char *client_id;
        int ping = 0;
        int i = 0;

	/* Receive message from client */
	while(!leave) {
                if(ping)
                {
                   /* if(handlePing(client_id, clntSock))
                    {
                        printf("Ping Failed\n");
                        close(clntSock);
                        return;
                    }*/
                }

                while(i < 5000) {i++;} //Wait between each message 
                i = 0;

                msg = receiveData(clntSock);

    //            printf("Message Contents:\nType: %d\nID: %s\nID_LEN: %d\n",
     //                   msg.type, msg.client_id, msg.id_len);
      //          printf("Length: %d\nData: %s\n", msg.length, msg.data);
//                continue;
                
                if((client_id = (char *)(malloc(sizeof(char) * msg.id_len))) ==
                        NULL)
                {
                    printf("Malloc Failed\n");
                    close(clntSock);
                    return;
                }
                strcpy(client_id, msg.client_id);

		//Decode incoming message and call appropriate handler
		switch(msg.type) {
			case(MESSAGE_UPDATE):
			{
	                        printf("UPDATE\n");			
				handleUpdate(msg.client_id, msg.data);
				break;
			}
			case(MESSAGE_FRIENDS):
			{
	                        printf("FRIENDS\n");			
				handleFriends(msg.client_id, msg.data,
                                        clntSock);
				break;
			}
			case(MESSAGE_HISTORY):
			{
	                        printf("HISTORY\n");			
				handleHistory(msg.client_id, clntSock);
				break;
			}
			case(MESSAGE_LEAVE):
			{
	                        printf("LEAVE\n");			
				handleLeave(msg.client_id);
				leave = 1;
				break;
			}
			case(MESSAGE_CHECKID):
			{
	                        printf("CHECKID\n");			
                                pthread_mutex_lock(&file_lock);
                                handleCheckId(msg.client_id, clntSock);
                                pthread_mutex_unlock(&file_lock);
                                ping = 1;
				break;
                        }
                        case(MESSAGE_INVALID):
                        {
	                    printf("INVALID\n");	
                            close(clntSock);
                            leave = 1;
                            break;
                        }
			default:
				printf("Unknown message type");
		}
	}

	close(clntSock);
}

int handleCheckId(char *client_id, int sock)
{
   Message msg;
   char *temp;
   char status;
   char *id;
   //Setup the file
   FILE *file;
   file = fopen("data.txt","r"); 

   if((msg.client_id = (char *)(malloc(sizeof(char) * strlen(client_id)))) == NULL)
   {
       printf("Unable to malloc space for the client's id\n");
       return -1;
   }

   if((msg.data = (char *)(malloc(sizeof(char)))) == NULL)
   {
       printf("Malloc Failed\n");
       return -1;
   }
   
   /*if((temp = (char *)(malloc(sizeof(char) * MAXLINELENGTH))) == NULL)
   {
       printf("Unable to malloc space for the client's id\n");
       return -1;
   }*/

   if(file == NULL)
   {
       msg.type = MESSAGE_IDAVAILABLE;
       msg.id_len = strlen(client_id);
       strcpy(msg.client_id, client_id);
       msg.length = 0;
       strcpy(msg.data, "");
       if(sendData(msg, sock))
       {
           printf("Send Failed\n");
           free(msg.client_id);
           free(temp);
           free(msg.data);
           return 1;
       }
       return 0;
   }

   temp = readLine(file);
   while(strcmp(temp, ""))
   {
       if(strlen(temp) < 3) //Ignore the line its too short
           continue;
       status = temp[0];
       temp = &temp[2];
       id = strtok(temp, " ");
       if(strcmp(id, client_id) == 0) //Found that Id already
       {
           if(status == 'a')
           {
               msg.type = MESSAGE_IDTAKEN;
           }
           else
           {
               msg.type = MESSAGE_IDAVAILABLE;
           }
         
           strcpy(msg.client_id, client_id);
           msg.id_len = strlen(msg.client_id);
           msg.length = 0;
           strcpy(msg.data, "");
  
           sendData(msg, sock);
           fclose(file);
           return 0; 
       }

       temp = readLine(file);
   }
   
   fclose(file);
   //free(temp);

   msg.type = MESSAGE_IDAVAILABLE;
   strcpy(msg.client_id, client_id);
   msg.id_len = strlen(msg.client_id);
   msg.length = 0;
   strcpy(msg.data, "");
   
   if(sendData(msg, sock))
   {
       printf("Error Occured during send\n");
       free(msg.client_id);
       free(msg.data);
       return 1;
   }

//   free(msg.client_id);
//   free(msg.data);
   return 0;
}

int handleUpdate(char *client_id, char *gps)
{
    int val;
    pthread_mutex_lock(&file_lock);
    val = replaceLine(client_id, gps);
    pthread_mutex_unlock(&file_lock);
    return val;
}

int handleFriends(char *client_id, char *friend_list, int sock)
{
    char *next_friend;
    Message msg;
    char *temp;
    int index = 0;

    msg.type = MESSAGE_FRIENDS;

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

    strcpy(msg.client_id, client_id);
    msg.id_len = strlen(msg.client_id);

    //printf("friends: %s\n", friend_list);

    next_friend = getFriend(friend_list, index);
    while(strcmp(next_friend, ""))
    { 
        index++;
        //printf("next_friend: %s\n", next_friend);
        pthread_mutex_lock(&file_lock);
        temp = getGPS(next_friend, MESSAGE_FRIENDS);
        pthread_mutex_unlock(&file_lock);
        
        strcat(msg.data, temp);
        next_friend = getFriend(friend_list, index);
    }

    msg.length = strlen(msg.data);
//    printf("length: %d Data: %s\n", msg.length, msg.data);

    if(sendData(msg, sock))
    {
        printf("Error Occured during Send\n");
        return 1;
    }

    return 0;
}

char *getFriend(char *friend_list, int index)
{
    int count = 0, i = 0, j = 0;
    char c;
    char *temp;

    if((temp = (char *)(malloc(sizeof(char) * strlen(friend_list)))) == NULL)
    {
        printf("Malloc Failed\n");
        return "";
    }
    strcpy(temp, "");

    while(count < index && i < strlen(friend_list))
    {
        c = friend_list[i];
        
        if(c == '\n') //Found the end of the first id
            count++;
        i++;
    }

    //get the actual name
    for(; i < strlen(friend_list); i++)
    {
        c = friend_list[i];
      //  printf("Char: %c Int: %d\n", c, c);

        if(c < 33) //Then the character is invalid
            break;

        temp[j] = c;
        j++;
    }

    temp[j] = 0;

    //printf("Return Val: %s\n", temp);

    return temp;
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
   //     free(msg.client_id);
        return 1;
    }
    
    msg.type = MESSAGE_HISTORY;
    strcpy(msg.client_id, client_id);
    msg.id_len = strlen(msg.client_id);

    pthread_mutex_lock(&file_lock);
    msg.data = getGPS(client_id, MESSAGE_HISTORY);
    pthread_mutex_unlock(&file_lock);
    
    msg.length = strlen(msg.data);

    if(sendData(msg, sock))
    {
        printf("Error Occured during sendData\n");
//        free(msg.client_id);
 //       free(msg.data);
        return 1;
    }

    //free(msg.client_id);
    //free(msg.data);
    return 0;
}

/* This function simply removes the client from the file it does not 
 * close the connection that has to be done in*/
int handleLeave(char *client_id)
{
    int val;
    pthread_mutex_lock(&file_lock);
    val = replaceLine(client_id, NULL);
    pthread_mutex_unlock(&file_lock);
}

int handlePing(char *client_id, int sock)
{
    Message msg;

    msg.type = MESSAGE_PING;
    msg.id_len = strlen(client_id);

    if((msg.client_id = (char *)(malloc(sizeof(char) * msg.id_len))) == NULL)
    {
        printf("Malloc Failed\n");
        return 1;
    }
    strcpy(msg.client_id, client_id);
   
    msg.length = 0;

    if((msg.data = (char *)(malloc(sizeof(char) * (msg.length+1)))) == NULL)
    {
        printf("Malloc Failed\n");
        free(msg.client_id);
        return 1;
    }
    strcpy(msg.data, "");

    return sendData(msg, sock);
}

char *getGPS(char *id, int type)
{
    char *line, *lat, *lon;
    FILE *fp;
    char *saved;
    char *temp;
    int i = 0;
    int offset = 0;

    fp = fopen("data.txt", "r");

    if((saved = (char *)(malloc(sizeof(char) * MAXLINELENGTH))) == NULL)
    {
        printf("Unable to malloc space\n");
        return "";
    }
   
    if(fp == NULL) //File is not there
    {
        sprintf(line, "%s: Not Found\n", id);
        return line;
    }
   
    line = readLine(fp);
    while(strcmp(line, ""))
    {
        if(strlen(line) < 3) //Ignore the line its to short
            continue;

        strcpy(saved, line);

       // printf("Line: %s", line);

        temp = &line[2]; //remove the status field
        temp = strtok(temp, " "); //get the first token

        if(strcmp(temp, id) == 0)
        {
            printf("MATCH\n");
            if(type == MESSAGE_FRIENDS)
            {
                lat = strtok(NULL, " "); //get the latitude
                lon = strtok(NULL, " "); //get the longitude
                //printf("Return Value: %s: %s %s", id, lat, lon);
                sprintf(line, "%s: %s %s", id, lat, lon);
            }
            else if(type == MESSAGE_HISTORY)
            {
                //remove the status field
                strcpy(line, &(saved[2]));
            }
            return line;
        }

        for(i = 0; i < strlen(line); i++) //Clear the line
        {
            line[i] = 0;
        }

        line = readLine(fp);
    }

    sprintf(line, "%s: Not Found\n", id);
    return line;
}

int replaceLine(char *client_id, char *gps)
{
   //TODO: Debug this method
   //Setup the files
   FILE *in;
   in = fopen("data.txt","r"); //fprintf(file,"%s","To write");
   FILE *out;
   out = fopen("temp.txt", "w");

   char *temp;
   char *line;
   char *saved;
   char *id;
   int found = 0;

   if((temp = (char *)(malloc(sizeof(char) * MAXLINELENGTH))) == NULL)
   {
        printf("Unable to malloc space for the file lines\n");
        return 1;
   }
   if((saved = (char *)(malloc(sizeof(char) * MAXLINELENGTH))) == NULL)
   {
        printf("Unable to malloc space for the file lines\n");
        return 1;
   }

   if(in == NULL)
   {    
       sprintf(temp, "a %s %s\n", client_id, gps);
       fputs(temp, out); //Write the line
       fclose(in);
       fclose(out);

       //Replace the old file with the new one
       remove("data.txt");
       rename("temp.txt", "data.txt");
       return 0;
   }

   line = readLine(in);
   while(strcmp(line, ""))
   {
      // printf("Line: %s", line);
       if(strlen(line) < 3) //skip the line too short
           continue;

       strcpy(saved, line);
       temp = &line[2];
       id = strtok(temp, " "); //get the first token as the id

       if(strcmp(id, client_id) == 0) //Found that Id already
       {
           printf("Line: %s", saved);
           printf("Old GPS: %s", &(saved[2+strlen(id)]));

           if(gps != NULL) //make the new line
           {
               sprintf(temp, "a %s %s %s", client_id, gps, &(saved[2+strlen(id)]));
           }
           else //this was a leave clear the line
           {
               sprintf(temp, "i %s %s", client_id, &(saved[2+strlen(id)]));
           }
           found = 1;          
       }
       printf("Out: %s", temp);
       fputs(temp, out); //Write the line (edited or not)

       line = readLine(in);
   }
   
   if(!found && gps != NULL)
   {
       sprintf(temp, "a %s %s\n", client_id, gps);
       printf("Out: %s", temp);
       fputs(temp, out); //Write the line
   }

   fclose(in);
   fclose(out);

   //Replace the old file with the new one
   remove("data.txt");
   rename("temp.txt", "data.txt");

   return 0;
}


char *readLine(FILE *fp)
{
    char *line;
    char c = 0;
    int i = 0;

    if((line = (char *)(malloc(sizeof(char) * MAXLINELENGTH))) == NULL)
    {
        printf("Malloc Failed\n");
        return NULL;
    }
    strcpy(line, "");

    c = fgetc(fp);
    while(c != '\n' && c != EOF && i < MAXLINELENGTH - 1) //Read the whole line as characters
    {
      line[i] = c;
      c = fgetc(fp);
      i++;
    }

    if(i != 0) //Add an endline if the line is not an empty line
        line[i] = '\n';

    return line;
}

/*
 * Sends the data (serialized) to the client 
 */
int sendData(Message msg, int sock)
{
    printf("Reached sendData\n");
    char *sendBuf;
    char *temp;
    int i;
    
    if((temp = (char *)(malloc(sizeof(char) * 9))) == NULL)
    {
        printf("Malloc failed\n");
        return 1;
    }

    //Send the type
    if((sendBuf = (char *)(malloc(sizeof(char) * 9))) == NULL)
    {
        printf("Malloc failed\n");
        return 1;
    }

    //Fill the size with 0's until size of int - temp is full
    //then append temp
    sprintf(temp, "%d", msg.type);
    printf("Type: %s len: %d\n", temp, strlen(temp));
    sprintf(sendBuf, "");
    for(i = 0; i < 9 - strlen(temp); i++) 
    {
        strcat(sendBuf, "0"); 
    }
    strcat(sendBuf, temp);
    printf("type: %d buf: %s\n", msg.type, sendBuf);
  
    if(executeSend(sendBuf, sock))
    {
        printf("send() failed\n");
        return 1;
    }
    free(sendBuf);

    //Send the id length
    if((sendBuf = (char *)(malloc(sizeof(char) * 9))) == NULL)
    {
        printf("Malloc failed\n");
        return 1;
    }
    
    //Fill the size with 0's until size of int - temp is full
    //then append temp
    sprintf(temp, "%d", msg.id_len);
    printf("ID_len: %s len: %d\n", temp, strlen(temp));
    sprintf(sendBuf, "");
    for(i = 0; i < 9 - strlen(temp); i++) 
    {
        strcat(sendBuf, "0"); 
    }
    strcat(sendBuf, temp);
    printf("ID length: %d buf: %s\n", msg.id_len, sendBuf);
    
    if(executeSend(sendBuf, sock))
    {
        printf("send() failed\n");
        return 1;
    }
    free(sendBuf);
    
    //Send the client id
    if((sendBuf = (char *)(malloc(sizeof(char) * strlen(msg.client_id)))) == NULL)
    {
        printf("Malloc failed\n");
        //Send 3 empty messages because the client expects 4 messages
        return 1;
    }
    sprintf(sendBuf, "%s", msg.client_id);
    printf("id: %s buf: %s\n", msg.client_id, sendBuf);
    if(executeSend(sendBuf, sock))
    {
        printf("send() failed\n");
        //Send 3 empty messages because the client expects 4 messages
        return 1;
    }
    free(sendBuf);

    //Send the length
    if((sendBuf = (char *)(malloc(sizeof(char) * 9))) == NULL)
    {
        printf("Malloc failed\n");
        //Send 2 empty messages because the client expects 4 messages
        return 1;
    }
    //Fill the size with 0's until size of int - temp is full
    //then append temp
    sprintf(temp, "%d", msg.length);
    printf("temp: %s len: %d\n", temp, strlen(temp));
    sprintf(sendBuf, "");
    for(i = 0; i < 9 - strlen(temp); i++) 
    {
        strcat(sendBuf, "0"); 
    }
    strcat(sendBuf, temp);
    printf("length: %d buf: %s\n", msg.length, sendBuf);
    
    if(executeSend(sendBuf, sock))
    {
        printf("send() failed\n");
        //Send 2 empty messages because the client expects 4 messages
        return 1;
    }
    free(sendBuf);
    
    //Send the data
    if(msg.length > 0)
    {
        if((sendBuf = (char *)(malloc(sizeof(char) * (msg.length+1)))) == NULL)
        {
            printf("Malloc failed\n");
            return 1;
        }
        sprintf(sendBuf, "%s", msg.data);
        printf("data: %s buf: %s\n", msg.data, sendBuf);
        if(executeSend(sendBuf, sock))
        {
            printf("send() failed\n");
            return 1;
        }
        free(sendBuf);
    }

    return 0;
}

int executeSend(char *buf, int sock)
{
    ssize_t numBytes = 0;
    int bufLen;
    
    //Send the msg
    bufLen = strlen(buf);
    numBytes = send(sock, buf, bufLen, 0);
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

Message receiveData(int sock)
{
    Message msg;
    char *buf;
    int i = 0;

/*    char *temp;
    
    if((temp = (char *)(malloc(sizeof(char) * 9))) == NULL)
    {
        printf("Malloc failed\n");
        msg.type = MESSAGE_INVALID;
        return msg;
    }
    
    //for all the integer fields use this size
    for(i = 0; i < 9; i++)
    {
        strcat(temp, "0");
    }

    int size = strlen(temp);
*/
    //get the type
    buf = executeReceive(sock, 9);
    printf("buf: %s\n", buf);
    if(buf == "")
    {
        msg.type = MESSAGE_INVALID;
        return msg;
    }
    msg.type = atoi(buf);

    if(msg.type == MESSAGE_INVALID)
    {
        return msg;
    }

    //get the id length
    buf = executeReceive(sock, 9);
    printf("buf: %s\n", buf);
    if(buf == "")
    {
        msg.type = MESSAGE_INVALID;
        return msg;
    }
    msg.id_len = atoi(buf);
   
    if(msg.id_len == 0)
    {
        printf("Invalid ID\n");
        msg.type = MESSAGE_INVALID;
        return msg;
    }

    //get the id
    buf = executeReceive(sock, msg.id_len);
    printf("buf: %s\n", buf);
    if(buf == "")
    {
        msg.type = MESSAGE_INVALID;
        return msg;
    }
    
    if((msg.client_id = (char *)(malloc(sizeof(char) * strlen(buf)))) == NULL)
    {
        printf("Malloc failed\n");
        msg.type = MESSAGE_INVALID;
    }
    
    if(msg.type == MESSAGE_INVALID)
    {
        return msg;
    }

    for(i = msg.id_len; i < strlen(buf); i++) //Clear any extraneous stuff 
    {
        buf[i] = 0;
    }
    strcpy(msg.client_id, buf);

    
    //get the length
    buf = executeReceive(sock, 9);
    printf("buf: %s\n", buf);
    if(buf == "")
    {
        msg.type = MESSAGE_INVALID;
        return msg;
    }
    msg.length = atoi(buf);

    //get the data
    if((msg.data = (char *)(malloc(sizeof(char) *(msg.length+1)))) == NULL)
    {
        printf("malloc failed\n");
        msg.type = MESSAGE_INVALID;
        return msg;
    }
   
    if(msg.length > 0)
    {
        buf = executeReceive(sock, msg.length);
        printf("buf: %s\n", buf);
        if(buf == "")
        {
            msg.type = MESSAGE_INVALID;
            return msg;
        }
        for(i = msg.length; i < strlen(buf); i++) //Clear any extraneous stuff 
        {
                buf[i] = 0;
        }
        strcpy(msg.data, buf);
    }
    else
    {
        strcpy(msg.data, "");
    }

    return msg;
}

char *executeReceive(int sock, int size)
{
    char *recvBuf;
    unsigned int totalBytesRcvd = 0;
    ssize_t numBytes = 0;
   
    printf("Reached executeReceive()\n");

    if((recvBuf = (char *)(malloc(sizeof(char) * size))) == NULL)
    {
        printf("Malloc failed\n");
        return "";
    }

    while(totalBytesRcvd < size)
    {
        numBytes = recv(sock, &(recvBuf[totalBytesRcvd]), size - totalBytesRcvd, 0);
        printf("Received %d\n", numBytes);

        if(numBytes < 0)
        {
            printf("recv() failed\n");
            return "";
        }

        totalBytesRcvd += numBytes;
        printf("Received %d of %d bytes\n", totalBytesRcvd, size);

        if(numBytes == 0)
            break;
    }
   
    return recvBuf;
}
