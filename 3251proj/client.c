#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>	  /* for socket(), connect(), send(), and recv() */
#include <arpa/inet.h>	  /* for sockaddr_in and inet_addr() */
#include <unistd.h>
#include "client.h"
#include "typedef.h"

int main(int argc, char **argv)
{
    int input;
    int connected = 0;
    char my_id[MAXIDLEN]; //The id of this client 
    int valid = 0;
    printf("Enter a user Id:\n");

    while(scanf("%s", my_id) != 1)
    {
        while(getchar() != '\n');
        printf("Invalid Input\n");
    }
    
    while(1)
    {
        displayMenu();
        
        if(connected) //ping the server
            handlePing(my_id);

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
               connected = handleConnect(my_id);
         
               break;
            }
            case 1:
            {   
               if(connected)
                       handleUpdate(my_id);
               else
                   printf("Not connected to the server\n");
               
               break;
            }
            case 2:
            {   
               if(connected)
                   handleFriends(my_id);
               else
                   printf("Not connected to the server\n");
               
               break;
            }
            case 3:
            {   
               if(connected)
                   handleHistory(my_id);
               else
                   printf("Not connected to the server\n");
               
               break;
            }
            case 4:
            {   
               if(connected)
                   connected = handleLeave(my_id);
               else
                   printf("Not connected to the server\n");
               
               break;
            }
            case 5:
            {    
               if(connected)
                   connected = handleLeave(my_id);
              
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
int handleConnect(char *id)
{
    //TODO: handle connect
    return 1;
}

/*
 * Gets the new GPS coordinates from the user and sends the update to the server
 *      Waits for the server's acknowledgement
 */
void handleUpdate(char *id)
{
    int lat_deg, lat_min, lat_sec, lon_deg, lon_min, lon_sec;
    char lat_dir, lon_dir;
    int valid = 0;

    //Get Latitude
    while(!valid)
    {
        printf("Enter Latitude in degrees minutes seconds N/S:\n");
        while(scanf("%d %d %d %c", &lat_deg, &lat_min, &lat_sec, &lat_dir) != 4)
        {
                while(getchar() != '\n');
                printf("Invalid Input\n");
                printf("Enter Latitude in degrees minutes seconds N/S:\n");
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
        printf("Enter Longitude in degrees minutes seconds E/W:\n");
        while(scanf("%d %d %d %c", &lon_deg, &lon_min, &lon_sec, &lon_dir) != 4)
        {
                while(getchar() != '\n');
                printf("Invalid Input\n");
                printf("Enter Longitude in degrees minutes seconds E/W:\n");
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
void handleFriends(char *id)
{
    char friends[MAXNUMREQUESTS][MAXIDLEN];
    char next_friend[MAXIDLEN];
    int index = 0;

    while(1)
    {
        printf("Enter the next Id (q to quit):\n");
        while(scanf("%s", next_friend) != 1)
        {
            while(getchar() != '\n');
            printf("Invalid Input\n");
            printf("Enter the next Id (q to quit)\n");
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
 * Requests the list of messages sent (pings are ignored) from the server
 *      Waits for the server's answer
 */
void handleHistory(char *id)
{
}

/*
 * Closes the connection to the server
 *      Returns 0 if it succeeds -1 otherwise
 */
int handleLeave(char *id)
{
    return 0;
}

/*
 * Pings the server
 *     Used so the server can handle non graceful leaves
 */
void handlePing(char *id)
{
}
