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

/* Function Prototypes */
void name_changer();

/* The main function */
int main(int argc, char *argv[])
{

    int serverSock;			/* Server Socket */
    int clientSock;			/* Client Socket */
    struct sockaddr_in changeServAddr;	/* Local address */
    struct sockaddr_in changeClntAddr;	/* Client address */
    unsigned short changeServPort;	/* Server port */
    unsigned int clntLen;		/* Length of address data struct */

    int i;				/* Declaring a counter variable*/
    char nameBuf[BUFSIZE];		/* Buff to store name from client */
    unsigned char resultBuf[SHALENGTH]; /* Buff to store change result */
    char answerBuf[HEXLENGTH];		/* Buff contains formatted answer */
    int rcvlen;
    unsigned short servPort = 4000;

    /* Create new TCP Socket for incoming requests*/
    if((serverSock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)
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


