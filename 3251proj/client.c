#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>	  /* for socket(), connect(), send(), and recv() */
#include <arpa/inet.h>	  /* for sockaddr_in and inet_addr() */
#include <unistd.h>
#include "typedef.h"
#include "client.h"
    
int clientSock;		    /* socket descriptor */
struct sockaddr_in serv_addr;   /* The server address */
char *my_id;

int main(int argc, char **argv)
{
    int input;
    int connected = 0;
    int valid = 0;
    int errorOccured = 0;

    if((my_id = (char *)(malloc(sizeof(char) * MAXIDLEN))) == NULL)
    {
        printf("Unable to malloc space for id\n");
        return;
    }
    
    while(1)
    {
        displayMenu();
        
        if(connected) //ping the server
           errorOccured = handlePing();

        if(errorOccured)
            connected = 0;

        while(scanf("%d", &input) != 1)
        {
            while(getchar() != '\n');
            printf("Invalid Input\n");
            displayMenu();
        }

        switch(input)
        {
            case 0:
            {
               errorOccured = handleConnect();

               if(!errorOccured)
                   connected = 1;

               break;
            }
            case 1:
            {   
               if(connected)
                   errorOccured = handleUpdate();
               else
                   printf("Not connected to the server\n");
               
               if(errorOccured)
                   connected = 0;

               break;
            }
            case 2:
            {   
               if(connected)
                   errorOccured = handleFriends();
               else
                   printf("Not connected to the server\n");
               
               if(errorOccured)
                   connected = 0;
               
               break;
            }
            case 3:
            {   
               if(connected)
                   errorOccured = handleHistory();
               else
                   printf("Not connected to the server\n");
               
               if(errorOccured)
                   connected = 0;
               
               break;
            }
            case 4:
            {   
               if(connected)
                   errorOccured = handleLeave();
               else
                   printf("Not connected to the server\n");
               
               connected = 0;

               break;
            }
            case 5:
            {    
               if(connected)
                   errorOccured = handleLeave();
              
               connected = 0;

               return;
            }
            default: 
            {
               printf("Unknwon Option Specified\n");
               break;
            }
        }
    }
}

/*
 * Displays the options menu
 */
void displayMenu()
{
    printf("(0) Connect:\n");
    printf("(1) Send Coordinates:\n");
    printf("(2) Get Coordinates:\n");
    printf("(3) History:\n");
    printf("(4) Leave:\n");
    printf("(5) Quit:\n");
}

/*
 * Connects to the server
 *      Returns 1 if it succeeds 0 otherwise
 */
int handleConnect()
{
    char *servIP;
    in_port_t servPort;
    int rtnVal = 0;
    Message send_msg;  //The check id message
    Message recv_msg; //The reply to the check id message

    /* Create a new TCP socket*/
    if((clientSock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)
    {
        printf("socket() failed\n");
        return 1;
    }
    
    /* Construct the server address structure */
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;

    /* Get the server's ip address from the user */
    if((servIP = (char *)(malloc(sizeof(char) * 16))) == NULL)//16 chars max ip length 
    {
        printf("Unable to malloc space for the server's ip\n");
        return 1;
    }
    
    /* Loop until the user enters a valid ip or inet_pton fails */
    while(rtnVal == 0)
    {
        printf("Enter the server's IP address: ");
        while(scanf("%s", servIP) != 1)
        {
            while(getchar() != '\n');
            printf("Invalid Input\n");
        }

        //Set the serv struct's ip to the value
        rtnVal = inet_pton(AF_INET, servIP, &serv_addr.sin_addr.s_addr);

        if(rtnVal == 0)
            printf("Invalid IP address\n");
    }

    if(rtnVal < 0)
    {
        printf("inet_pton() failed\n");
        return 1;
    }
     
    /* Get the server's port from the user */
    printf("Enter the server's port: ");
    while(scanf("%d", &servPort) != 1)
    {
        while(getchar() != '\n');
        printf("Invalid Input\n");
    }
    serv_addr.sin_port = htons(servPort);

    /* Establish connecction to the server */
    if((connect(clientSock, (struct sockaddr *)&serv_addr, sizeof(serv_addr))) < 0)
    {
        printf("connect() failed\n");
        return 1;
    }
    
    printf("Enter your user Id: ");
    while(scanf("%s", my_id) != 1)
    {
        while(getchar() != '\n');
        printf("Invalid Input\n");
    }

    /* The message has no data just the id of this client */
    send_msg.type = MESSAGE_CHECKID;
    send_msg.length = 0;

    if((send_msg.client_id = (char *)(malloc(sizeof(char) * MAXIDLEN))) == NULL)
    {
        printf("Unable to malloc space for the send's client id\n");
        close(clientSock);
        clientSock = -1;
        return 1;
    }

    strcpy(send_msg.client_id, my_id);
    send_msg.data = NULL;

    if(sendData(send_msg))
    {
        printf("Error Sending your id to the server\n");
        close(clientSock);
        clientSock = -1;
        free(send_msg.client_id);
        return 1;
    }

    recv_msg = receiveData();

    if(recv_msg.type == MESSAGE_INVALID) //An error occured
    {
        printf("Error Receiving the server's response\n");
        close(clientSock);
        clientSock = -1;
        free(send_msg.client_id);
        return 1;
    }

    while(recv_msg.type != MESSAGE_IDAVAILABLE)
    {
        printf("That Id is already taken.\n");
        printf("Enter your user Id: ");
        while(scanf("%s", my_id) != 1)
        {
            while(getchar() != '\n');
            printf("Invalid Input\n");
        }
    
        strcpy(send_msg.client_id, my_id);
        if(sendData(send_msg))
        {
            printf("Error Sending your id to the server\n");
            close(clientSock);
            clientSock = -1;
            free(send_msg.client_id);
            return 1;
        }

        recv_msg = receiveData();
        if(recv_msg.type == MESSAGE_INVALID) //An error occured

        {
            printf("Error Receiving the server's response\n");
            close(clientSock);
            clientSock = -1;
            free(send_msg.client_id);
            return 1;
        }
    }

    //free the client_id
    free(send_msg.client_id);
 
    return 0;
}

/*
 * Gets the new GPS coordinates from the user and sends the update to the server
 *      Waits for the server's acknowledgement
 */
int handleUpdate()
{
    int lat_deg, lat_min, lat_sec, lon_deg, lon_min, lon_sec;
    char lat_dir, lon_dir;
    int valid = 0;
    Message send_msg;

    //Get Latitude
    while(!valid)
    {
        printf("Enter Latitude in degrees minutes seconds N/S: ");
        while(scanf("%d %d %d %c", &lat_deg, &lat_min, &lat_sec, &lat_dir) != 4)
        {
                while(getchar() != '\n');
                printf("Invalid Input\n");
                printf("Enter Latitude in degrees minutes seconds N/S: ");
        }

        valid = (lat_deg >= 0 && lat_deg <= 90) && 
            (lat_min >= 0 && lat_min < 60) &&
            (lat_sec >= 0 && lat_sec < 60) &&
            (lat_dir == 'N' || lat_dir == 'n' || lat_dir == 'S' || lat_dir =='s');

        if(!valid)
            printf("Invalid Latitude\n");
    }

    //Get Longitude
    valid = 0;
    while(!valid)
    {
        printf("Enter Longitude in degrees minutes seconds E/W: ");
        while(scanf("%d %d %d %c", &lon_deg, &lon_min, &lon_sec, &lon_dir) != 4)
        {
                while(getchar() != '\n');
                printf("Invalid Input\n");
                printf("Enter Longitude in degrees minutes seconds E/W: ");
        }
        
        valid = (lon_deg >= 0 && lon_deg <= 180) && 
            (lon_min >= 0 && lon_min < 60) &&
            (lon_sec >= 0 && lon_sec < 60) &&
            (lon_dir == 'E' || lon_dir == 'e' || lon_dir == 'W' || lon_dir =='w');

        if(!valid)
            printf("Invalid Longitude\n");
    }

    //printf("Latitude:%d.%d.%d%c Longitude:%d.%d.%d%c\n", lat_deg, lat_min,
    //        lat_sec, lat_dir, lon_deg, lon_min, lon_sec, lon_dir);
    
    /*Make the Message*/
    send_msg.type = MESSAGE_UPDATE;
 
    if((send_msg.client_id = (char *)(malloc(sizeof(my_id)))) == NULL)
    {
        printf("Unable to malloc space for the message's client id\n");
        close(clientSock);
        clientSock = -1;
        return 1;
    }
    strcpy(send_msg.client_id, my_id);
    
    if((send_msg.data = (char *)(malloc(sizeof(char) * 20))) == NULL)
    {
        printf("Unable to malloc space for the message's data\n");
        close(clientSock);
        clientSock = -1;
        free(send_msg.client_id);
        return 1;
     }

    sprintf(send_msg.data, "%d.%d.%d%c %d.%d.%d%c", lat_deg, lat_min, lat_sec,
            lat_dir, lon_deg, lon_min, lon_sec, lon_dir);

    send_msg.length = strlen(send_msg.data);

    if(sendData(send_msg))
    {
        printf("Error Occured while sending update message\n");
        close(clientSock);
        clientSock = -1;
        free(send_msg.client_id);
        free(send_msg.data);
        return 1;
    }

    free(send_msg.client_id);
    free(send_msg.data);

    return 0;
}

/*
 * Gets the list of friends from the user and sends the request to the server
 *      Waits for the server's answer
 */
int handleFriends()
{
    char friends[MAXNUMREQUESTS][MAXIDLEN];
    char next_friend[MAXIDLEN];
    int index = 0;
    Message send_msg;
    Message recv_msg;
    int i, j;

    while(1)
    {
        printf("Enter the next Id (q to quit): ");
        while(scanf("%s", next_friend) != 1)
        {
            while(getchar() != '\n');
            printf("Invalid Input\n");
            printf("Enter the next Id (q to quit): ");
        }

        //Exit on the quit
        if(next_friend[0] == 'q' || next_friend[0] == 'Q')
            break;

        strcpy(friends[index], next_friend);

    //    printf("Next Friend: %s\n", next_friend);
    //    printf("Friend[%d]: %s\n", index, friends[index]);
        index++;
    }

    /* Make the message */
    send_msg.type = MESSAGE_FRIENDS;

    if((send_msg.client_id = (char *)(malloc(sizeof(my_id)))) == NULL)
    {
        printf("Unable to malloc space for the message's client id\n");
        close(clientSock);
        clientSock = -1;
        return 1;
    }
    strcpy(send_msg.client_id, my_id);
    
    if((send_msg.data = (char *)(malloc(sizeof(char) * MAXNUMREQUESTS *
                        (MAXIDLEN + 1)))) == NULL)
    {
        printf("Unable to malloc space for message's data\n");
        close(clientSock);
        clientSock = -1;
        free(send_msg.client_id);
        return 1;
    }

    //Data is sent in form "ID\nID\nID\n..."
    index = 0;
    for(i = 0; i < MAXNUMREQUESTS; i++)
    {
        for(j = 0; j < MAXIDLEN; j++)
        {
            char c = friends[i][j];
            
            if(c < 32) //invalid character name is done
            {
                send_msg.data[index] = '\n';
                index++;
                break;
            }
            else
            {
                send_msg.data[index] = c;
                index++;
            }
        }
    }
    
    if(sendData(send_msg))
    {
        printf("Error Occured sending the data\n");
        close(clientSock);
        clientSock = -1;
        free(send_msg.client_id);
        free(send_msg.data);
        return 1;
    }

    recv_msg = receiveData();

    if(recv_msg.type == MESSAGE_INVALID)
    {
        printf("Error Occured receiving the server's response\n");
        close(clientSock);
        clientSock = -1;
        free(send_msg.client_id);
        free(send_msg.data);
        return 1;
    }

    //Data is received in form "ID GPS\nID GPS\nID ..."
    printf("%s", recv_msg.data);
    
    
    free(send_msg.client_id);
    free(send_msg.data);
    free(recv_msg.client_id);
    free(recv_msg.data);
   
    return 0;
}   

/*
 * Requests the list of previous gps locations from the server
 *      Waits for the server's answer
 */
int handleHistory()
{
    Message send_msg;
    Message recv_msg;

    /* Make the message */
    send_msg.type = MESSAGE_HISTORY;

    if((send_msg.client_id = (char *)(malloc(sizeof(my_id)))) == NULL)
    {
        printf("Unable to malloc space for the message's client id\n");
        close(clientSock);
        clientSock = -1;
        return 1;
    }
    strcpy(send_msg.client_id, my_id);
    
    send_msg.length = 0;
    send_msg.data = NULL;

    if(sendData(send_msg))
    {
        printf("Error Occured with sending history message\n");
        close(clientSock);
        clientSock = -1;
        free(send_msg.client_id);
        return 1;
    }
 
    recv_msg = receiveData();

    if(recv_msg.type == MESSAGE_INVALID)
    {
        printf("Error Occured with the server's response\n");
        close(clientSock);
        clientSock = -1;
        free(send_msg.client_id);
        return 1;
    }

    //Data is in form "GPS\nGPS\n..."
    printf("%s", recv_msg.data);

    free(send_msg.client_id);
    free(recv_msg.client_id);
    free(recv_msg.data);

    return 0;
}

/*
 * Closes the connection to the server
 *      Returns 0 if it succeeds 1 otherwise
 */
int handleLeave()
{
    Message send_msg;

    /* Make the message */
    send_msg.type = MESSAGE_LEAVE;

    if((send_msg.client_id = (char *)(malloc(sizeof(my_id)))) == NULL)
    {
        printf("Unable to malloc space for the message's client id\n");
        close(clientSock);
        clientSock = -1;
        return 1;
    }
    strcpy(send_msg.client_id, my_id);
    
    send_msg.length = 0;
    send_msg.data = NULL;

    if(sendData(send_msg))
    {
        printf("Error Occured with sending history message\n");
        close(clientSock);
        clientSock = -1;
        free(send_msg.client_id);
        return 1;
    }

    free(send_msg.client_id);

    return 0;
}

/*
 * Pings the server
 *     Used so the server can handle non graceful leaves
 */
int handlePing()
{
    Message send_msg;

    /* Make the message */
    send_msg.type = MESSAGE_PING;

    if((send_msg.client_id = (char *)(malloc(sizeof(my_id)))) == NULL)
    {
        printf("Unable to malloc space for the message's client id\n");
        close(clientSock);
        clientSock = -1;
        return 1;
    }
    strcpy(send_msg.client_id, my_id);
    
    send_msg.length = 0;
    send_msg.data = NULL;

    if(sendData(send_msg))
    {
        printf("Error Occured with sending history message\n");
        close(clientSock);
        clientSock = -1;
        free(send_msg.client_id);
        return 1;
    }

    free(send_msg.client_id);
    
    return 0;
}


/*
 * Sends the data (serialized) to the server 
 */
int sendData(Message msg)
{
    ssize_t numBytes = 0;
    int bufLen;

    //char sendBuf[SENDBUFSIZE];


    //Send the msg
    //sprintf(sendBuf, "Type:%d\nID:%s\nLength:%d\nData:%s\n", msg.type,
    //        msg.id, msg.length, msg.data);
    bufLen = strlen((char *)(&msg));
    numBytes = send(clientSock, (char *)(&msg), bufLen, 0);
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

/*
 * Receives the data from the server and returns it
 */
Message receiveData()
{
    Message *temp;
    Message msg;
    unsigned int totalBytesRcvd = 0;
    ssize_t numBytes = 0;
    char recvBuf[RECVBUFSIZE];
    char s[RECVBUFSIZE];
    char token[10];
    int i, j = 0;

    while(totalBytesRcvd < RECVBUFSIZE)
    {
        numBytes = recv(clientSock, &(recvBuf[totalBytesRcvd]), RECVBUFSIZE - 
                totalBytesRcvd, 0);

        if(numBytes < 0)
        {
            printf("recv() failed\n");
            msg.type = MESSAGE_INVALID;
            return msg;
        }

        totalBytesRcvd += numBytes;

        if(numBytes == 0)
            break;
    }

    temp = (Message *)(recvBuf);

    msg.type = temp->type;
    msg.length = temp->length;

    if((msg.client_id = (char *)(malloc(sizeof(temp->client_id)))) == NULL)
    {
        printf("Unable to malloc space for the received message\n");
        free(temp->client_id);
        free(temp->data);
        free(temp);
        msg.type = MESSAGE_INVALID;
        return msg;
    }

    strcpy(msg.client_id, temp->client_id);

    if((msg.data = (char *)(malloc(sizeof(temp->data)))) == NULL)
    {
        printf("Unable to malloc space for the received message\n");
        free(temp->client_id);
        free(temp->data);
        free(temp);
        msg.type = MESSAGE_INVALID;
        return msg;
    }

    strcpy(msg.data, temp->data);

    return msg;
}
