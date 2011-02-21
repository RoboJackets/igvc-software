#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>	  /* for socket(), connect(), send(), and recv() */
#include <arpa/inet.h>	  /* for sockaddr_in and inet_addr() */
#include <unistd.h>
#include "client.h"
#include "typedef.h"
    
int clientSock;		    /* socket descriptor */
struct sockaddr_in serv_addr;   /* The server address */
char *my_id;

int main(int argc, char **argv)
{
    int input;
    int connected = 0;
    int valid = 0;

    if((my_id = (char *)(malloc(sizeof(char) * MAXIDLEN))) == NULL)
    {
        printf("Unable to malloc space for id\n");
        return;
    }
    
    while(1)
    {
        displayMenu();
        
        if(connected) //ping the server
            handlePing();

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
               connected = handleConnect();
               break;
            }
            case 1:
            {   
               if(connected)
                       handleUpdate();
               else
                   printf("Not connected to the server\n");
               
               break;
            }
            case 2:
            {   
               if(connected)
                   handleFriends();
               else
                   printf("Not connected to the server\n");
               
               break;
            }
            case 3:
            {   
               if(connected)
                   handleHistory();
               else
                   printf("Not connected to the server\n");
               
               break;
            }
            case 4:
            {   
               if(connected)
                   connected = handleLeave();
               else
                   printf("Not connected to the server\n");
               
               break;
            }
            case 5:
            {    
               if(connected)
                   connected = handleLeave();
              
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
//    Message *send_msg;  //The check id message
//    Message *rcv_msg; //The reply to the check id message
    int available = 0; //whether the id is available or not

    /* Create a new TCP socket*/
    if((clientSock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)
    {
        printf("socket() failed\n");
        return 0;
    }
    
    
    /* Construct the server address structure */
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;

    /* Get the server's ip address from the user */
    if((servIP = (char *)(malloc(sizeof(char) * 16))) == NULL)//16 chars max ip length 
    {
        printf("Unable to malloc space for the server's ip\n");
        return 0;
    }
    
    /* Loop until the user enters a valid ip or inet_pton fails */
    while(rtnVal == 0)
    {
        printf("Enter the server's IP address:");
        while(scanf("%s", servIP) != 1)
        {
                while(getchar() != '\n');
                printf("Invalid Input\n");
        }

        //Set the serv struct's ip to the value
        rtnVal = inet_pton(AF_INET, servIP, &serv_addr.sin_addr.s_addr);
    }

    if(rtnVal < 0)
    {
        printf("inet_pton() failed\n");
        return 0;
    }
     
    /* Get the server's port from the user */
    printf("Enter the server's port:");
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
        return 0;
    }
    
    printf("Enter a user Id:");
    while(scanf("%s", my_id) != 1)
    {
        while(getchar() != '\n');
        printf("Invalid Input\n");
    }

  //  if((send_msg = (Message *)(malloc(sizeof(Message)))) == NULL)
  //  {
  //      printf("Unable to malloc space for the send_msg\n");
   //     close(clientSock); //close the connection
    //    return 0;
   // }

    /* The message has no data just the id of this client */
 //   send_msg->type = MESSAGE_CHECKID;
 //   send_msg->length = 0;
 //   send_msg->client_id = my_id;
 //   send_msg->data = NULL;

    //ssize_t numByetes = send(clientSock, 

    return 1;
}

/*
 * Gets the new GPS coordinates from the user and sends the update to the server
 *      Waits for the server's acknowledgement
 */
void handleUpdate()
{
    int lat_deg, lat_min, lat_sec, lon_deg, lon_min, lon_sec;
    char lat_dir, lon_dir;
    int valid = 0;

    //Get Latitude
    while(!valid)
    {
        printf("Enter Latitude in degrees minutes seconds N/S:");
        while(scanf("%d %d %d %c", &lat_deg, &lat_min, &lat_sec, &lat_dir) != 4)
        {
                while(getchar() != '\n');
                printf("Invalid Input\n");
                printf("Enter Latitude in degrees minutes seconds N/S:");
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
        printf("Enter Longitude in degrees minutes seconds E/W:");
        while(scanf("%d %d %d %c", &lon_deg, &lon_min, &lon_sec, &lon_dir) != 4)
        {
                while(getchar() != '\n');
                printf("Invalid Input\n");
                printf("Enter Longitude in degrees minutes seconds E/W:");
        }
        
        valid = (lon_deg >= 0 && lon_deg <= 180) && 
            (lon_min >= 0 && lon_min < 60) &&
            (lon_sec >= 0 && lon_sec < 60) &&
            (lon_dir == 'E' || lon_dir == 'e' || lon_dir == 'W' || lon_dir =='w');

        if(!valid)
            printf("Invalid Longitude\n");
    }

    printf("Latitude:%d.%d.%d%c Longitude:%d.%d.%d%c\n", lat_deg, lat_min,
            lat_sec, lat_dir, lon_deg, lon_min, lon_sec, lon_dir);
}

/*
 * Gets the list of friends from the user and sends the request to the server
 *      Waits for the server's answer
 */
void handleFriends()
{
    char friends[MAXNUMREQUESTS][MAXIDLEN];
    char next_friend[MAXIDLEN];
    int index = 0;

    while(1)
    {
        printf("Enter the next Id (q to quit):");
        while(scanf("%s", next_friend) != 1)
        {
            while(getchar() != '\n');
            printf("Invalid Input\n");
            printf("Enter the next Id (q to quit)");
        }

        //Exit on the quit
        if(next_friend[0] == 'q' || next_friend[0] == 'Q')
            break;

        strcpy(friends[index], next_friend);

        printf("Next Friend: %s\n", next_friend);
        printf("Friend[%d]: %s\n", index, friends[index]);
        index++;
    }

    //Handle server interaction
}   

/*
 * Requests the list of previous gps locations from the server
 *      Waits for the server's answer
 */
void handleHistory()
{
}

/*
 * Closes the connection to the server
 *      Returns 0 if it succeeds -1 otherwise
 */
int handleLeave()
{
    return 0;
}

/*
 * Pings the server
 *     Used so the server can handle non graceful leaves
 */
void handlePing()
{
}
