/** Function Prototypes */

/*Displays the menu of options to the user*/
void displayMenu(void);

/*Connects to the server*/
int handleConnect(void);

/*Updates the location*/
void handleUpdate(void);

/*Gets and displays friend location*/
void handleFriends(void);

/*Returns the history of this clients locations*/
void handleHistory(void);

/*Closes the connection*/
int handleLeave(void);

/*Checks to see whether this id is taken*/
int handleCheckId(void);

/*Pings the server for verification of connection*/
void handlePing(void);
