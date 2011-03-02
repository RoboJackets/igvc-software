/** Function Prototypes */

/*Displays the menu of options to the user*/
void displayMenu(void);

/*Connects to the server*/
int handleConnect(void);

/*Updates the location*/
int handleUpdate(void);

/*Gets and displays friend location*/
int handleFriends(void);

/*Returns the history of this clients locations*/
int handleHistory(void);

/*Closes the connection*/
int handleLeave(void);

/*Pings the server for verification of connection*/
int handlePing(void);

/*Sends the data (serialized) to the server*/
int sendData(Message msg);

/*Receives the data from the server and returns it*/
Message receiveData(void);
