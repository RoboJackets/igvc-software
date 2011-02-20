/** Function Prototypes */

/*Displays the menu of options to the user*/
void displayMenu(void);

/*Connects to the server*/
int handleConnect(char *id);

/*Updates the location*/
void handleUpdate(char *id);

/*Gets and displays friend location*/
void handleFriends(char *id);

/*Returns the history of this clients requests*/
void handleHistory(char *id);

/*Closes the connection*/
int handleLeave(char *id);

/*Checks to see whether this id is taken*/
int handleCheckId(char *id);

/*Pings the server for verification of connection*/
void handlePing(char *id);
