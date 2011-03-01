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

int executeSend(char *buf);
char *executeReceive(int size);

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
//        printf("Next Option\n");
        displayMenu();
        
        if(errorOccured)
            connected = 0;

        while(scanf("%d", &input) != 1)
        {
            while(getchar() != '\n');
            printf("Invalid Input\n");
            displayMenu();
        }

  //      printf("Input: %d\n", input);

        switch(input)
        {
            case 0:
            {
//               printf("CONNECT\n"); 
               if(!connected)
                   errorOccured = handleConnect();
               else
                   printf("Already Connected\n");

               if(!errorOccured)
                   connected = 1;
  //             printf("CONNECT\n"); 

               break;
            }
            case 1:
            {   
    //           printf("UPDATE\n"); 
               if(connected)
                   errorOccured = handleUpdate();
               else
                   printf("Not connected to the server\n");
               
               if(errorOccured)
                   connected = 0;
      //         printf("UPDATE\n"); 

               break;
            }
            case 2:
            {   
        //       printf("FRIENDS\n"); 
               if(connected)
                   errorOccured = handleFriends();
               else
                   printf("Not connected to the server\n");
               
               if(errorOccured)
                   connected = 0;
          //     printf("FRIENDS\n"); 
               
               break;
            }
            case 3:
            {   
            //   printf("HISTORY\n"); 
               if(connected)
                   errorOccured = handleHistory();
//                   errorOccured = 0;
               else
                   printf("Not connected to the server\n");
               
               if(errorOccured)
                   connected = 0;
              // printf("HISTORY\n"); 
               
               break;
            }
            case 4:
            {   
             //  printf("LEAVE\n"); 
               if(connected)
                   errorOccured = handleLeave();
               else
                   printf("Not connected to the server\n");
               
               connected = 0;
             //  printf("LEAVE\n"); 

               break;
            }
            case 5:
            {    
              // printf("QUIT\n"); 
               if(connected)
                   errorOccured = handleLeave();
              
               connected = 0;
//               printf("QUIT\n"); 

               return;
            }
            default: 
            {
               printf("Unknwon Option Specified\n");
               break;
            }
        }
    }

    return 0;
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
    unsigned short servPort = 4000;
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
    serv_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    serv_addr.sin_port = htons(servPort);

    /* Get the server's ip address from the user */
   /* if((servIP = (char *)(malloc(sizeof(char) * 16))) == NULL)//16 chars max ip length 
    {
        printf("Unable to malloc space for the server's ip\n");
        return 1;
    }*/
    
    /* Loop until the user enters a valid ip or inet_pton fails */
/*     while(rtnVal == 0)
     {
        printf("Enter the server's IP address: ");
        while(scanf("%s", servIP) != 1)
        {
            while(getchar() != '\n');
            printf("Invalid Input\n");
        }

        //Set the serv struct's ip to the value
          rtnVal = inet_pton(AF_INET, servIP, &serv_addr.sin_addr.s_addr);
  //      serv_addr.sin_addr.s_addr = inet_addr(servIP);
          if(rtnVal == 0)
             printf("Invalid IP address\n");
      }

      if(rtnVal < 0)
      {
          printf("inet_pton() failed\n");
          return 1;
      }
   

  */   
    /* Get the server's port from the user */
  /*  printf("Enter the server's port: ");
    while(scanf("%d", &servPort) != 1)
    {
        while(getchar() != '\n');
        printf("Invalid Input\n");
    }
    //serv_addr.sin_port = htons(servPort);
*/

    /* Establish connecction to the server */
    if((connect(clientSock, (struct sockaddr *)&serv_addr, sizeof(serv_addr))) < 0)
    {
        printf("connect() failed\n");
        return 1;
    }
    

    printf("Enter your user Id (No Spaces Allowed): ");
    while(scanf("%s", my_id) != 1 || strcmp(my_id, "q") == 0)
    {
        while(getchar() != '\n');
        printf("Invalid Input\n");

        if(strcmp(my_id, "q") == 0)
            printf("Id cannot be q\n");
        
        printf("Enter your user Id (No Spaces Allowed): ");
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
    send_msg.id_len = strlen(send_msg.client_id);
//    printf("ID: %s Length: %d\n", send_msg.client_id, send_msg.id_len);
    strcpy(send_msg.data, "");

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
        send_msg.id_len = strlen(send_msg.client_id);
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
    send_msg.id_len = strlen(send_msg.client_id);
    
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

    //Clear the friends array
    for(i = 0; i < MAXNUMREQUESTS; i++)
    {
        for(j = 0; j < MAXIDLEN; j++)
        {
            friends[i][j] = 0;
        }
    }

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
    send_msg.id_len = strlen(send_msg.client_id);
    
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
            
            if(c == 0) //invalid character name is done
            {
                if(j != 0)
                {
                    send_msg.data[index] = '\n';
                    index++;
                    break;
                }
                else //this id is not set skip it
                {
                   break;
                }
            }
            else
            {
                send_msg.data[index] = c;
                index++;
            }
        }
    }
    send_msg.length = strlen(send_msg.data);
    //printf("Length: %d Data: %s\n", send_msg.length, send_msg.data);
    
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
    printf("Friends Locations\n%s\n", recv_msg.data);
    
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

    if((send_msg.data = (char *)(malloc(sizeof(char)))) == NULL)
    {
        printf("Malloc Failed\n");
        close(clientSock);
        clientSock = -1;
        return 1;
    }
    strcpy(send_msg.client_id, my_id);
    send_msg.id_len = strlen(send_msg.client_id); 
    send_msg.length = 0;
    strcpy(send_msg.data, "");

    printf("Message Contents:\nType: %d\nId Len: %d\nId: %s\nLen: %d\nData: %s\n",
            send_msg.type, send_msg.id_len, send_msg.client_id, send_msg.length, 
            send_msg.data);

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

    printf("History: %s\n", recv_msg.data);

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

    if((send_msg.data = (char *)(malloc(sizeof(char)))) == NULL)
    {
        printf("Malloc Failed\n");
        close(clientSock);
        clientSock = -1;
        return 1;
    }

    strcpy(send_msg.client_id, my_id);
    send_msg.id_len = strlen(send_msg.client_id);
    send_msg.length = 0;
    strcpy(send_msg.data, "");

    if(sendData(send_msg))
    {
        printf("Error Occured with sending history message\n");
        close(clientSock);
        clientSock = -1;
        free(send_msg.client_id);
        return 1;
    }

    //free(send_msg.client_id);

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
    send_msg.id_len = strlen(send_msg.client_id);
    send_msg.length = 0;
    strcpy(send_msg.data, "");

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
    char *sendBuf;
    char *temp;
    int i;
    
    printf("send Data\n");
    printf("Message Contents:\nType: %d\nId Len: %d\nId: %s\nLen: %d\nData: %s\n",
            msg.type, msg.id_len, msg.client_id, msg.length, 
            msg.data);
   
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
    
    //Pad the string with 0's so the size is correct
    sprintf(temp, "%d", msg.type);
    sprintf(sendBuf, "");
    for(i = 0; i < 9 - strlen(temp); i++)
    {
        strcat(sendBuf, "0");
    }
    strcat(sendBuf, temp);

    //printf("Type: %d Buf: %s\n", msg.type, sendBuf);
    if(executeSend(sendBuf))
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
    
    //Pad the string with 0's so the size is correct
    sprintf(temp, "%d", msg.id_len);
    sprintf(sendBuf, "");
    for(i = 0; i < 9 - strlen(temp); i++)
    {
        strcat(sendBuf, "0");
    }
    strcat(sendBuf, temp);
    
    //printf("ID Length: %d Buf: %s\n", msg.id_len, sendBuf);
    if(executeSend(sendBuf))
    {
        printf("send() failed\n");
        return 1;
    }
    free(sendBuf);

    //Send the client id
    if((sendBuf = (char *)(malloc(sizeof(char) * strlen(my_id)))) == NULL)
    {
        printf("Malloc failed\n");
        return 1;
    }
    sprintf(sendBuf, "%s", msg.client_id);
    //printf("Id: %s Buf %s\n", msg.client_id, sendBuf);
    if(executeSend(sendBuf))
    {
        printf("send() failed\n");
        return 1;
    }
    free(sendBuf);

    //Send the length
    if((sendBuf = (char *)(malloc(sizeof(char) * 9))) == NULL)
    {
        printf("Malloc failed\n");
        return 1;
    }
    
    //Pad the string with 0's so the size is correct
    sprintf(temp, "%d", msg.length);
    sprintf(sendBuf, "");
    for(i = 0; i < 9 - strlen(temp); i++)
    {
            strcat(sendBuf, "0");
    }
    strcat(sendBuf, temp);
    
    //printf("length: %d buf: %s\n", msg.length, sendBuf);
    if(executeSend(sendBuf))
    {
        printf("send() failed\n");
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
        //printf("Data: %s buf: %s\n", msg.data, sendBuf);
        if(executeSend(sendBuf))
        {
            printf("send() failed\n");
            return 1;
        }
        free(sendBuf);
    }

    return 0;
}

int executeSend(char *buf)
{
    ssize_t numBytes = 0;
    int bufLen;
    
    //Send the msg
    bufLen = strlen(buf);
    numBytes = send(clientSock, buf, bufLen, 0);
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
    Message msg;
    char *buf;
    int i = 0;
    
    //get the type
    buf = executeReceive(9);
    //printf("buf: %s\n", buf);
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

    //get the id len
    buf = executeReceive(9);
    //printf("buf: %s\n", buf);
    if(buf == "")
    {
        msg.type = MESSAGE_INVALID;
        return msg;
    }
    msg.id_len = atoi(buf);
    
    if(msg.id_len == 0)
    {
        printf("Invalid Id\n");
        msg.type = MESSAGE_INVALID;
        return msg;
    }
    
    //get the id
    buf = executeReceive(msg.id_len);
    //printf("buf: %s\n", buf);
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

    for(i = msg.id_len; i < strlen(buf); i++) //Clear any extraneous data
    {
        buf[i] = 0;
    }
    strcpy(msg.client_id, buf);

    
    //get the length
    buf = executeReceive(9);
    //printf("buf: %s\n", buf);
    if(buf == "")
    {
        msg.type = MESSAGE_INVALID;
        return msg;
    }
    msg.length = atoi(buf);

    //get the data
    if((msg.data = (char *)(malloc(sizeof(char) * (msg.length+1)))) == NULL)
    {
        printf("malloc failed\n");
        msg.type = MESSAGE_INVALID;
        return msg;
    }

    if(msg.length > 0)
    {
        buf = executeReceive(msg.length);
        //printf("buf: %s\n", buf);
        if(buf == "")
        {
            msg.type = MESSAGE_INVALID;
            return msg;
        }
        for(i = msg.length; i < strlen(buf); i++) //Clear any extraneous data
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


char *executeReceive(int size)
{
    char *recvBuf;
    unsigned int totalBytesRcvd = 0;
    ssize_t numBytes = 0;
    
    if((recvBuf = (char *)(malloc(sizeof(char) * size))) == NULL)
    {
        printf("Malloc failed\n");
        return "";
    }

    while(totalBytesRcvd < size)
    {
        numBytes = recv(clientSock, &(recvBuf[totalBytesRcvd]), 
                size - totalBytesRcvd, 0);
        //printf("Received %d\n", numBytes);

        if(numBytes < 0)
        {
            printf("recv() failed\n");
            return "";
        }

        totalBytesRcvd += numBytes;
        //printf("Received %d of %d bytes\n", totalBytesRcvd, size);

        if(numBytes == 0)
            break;
    }

    return recvBuf;
}
