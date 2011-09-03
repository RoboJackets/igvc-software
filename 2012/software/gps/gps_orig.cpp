#include <stdio.h>   /* Standard input/output definitions */
#include<stdlib.h>

#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */




class GPS {

	struct coords {
	  float lat;
	  char lat_dir;
	  float lon;
	  char lon_dir;
	  float prev_lat;
	  float prev_lon;
	}latlon;
	
	float
	int filedesc;
	int msg_recieved;
	char buffer[1]; 
      	char *bufptr; 
	void open_port (void);
	void read_port (void);
	void write_port (void);
	void check_message(void);
	void print(void);
	int message_recieved(void);
	void parse_message(void);
	
	char message[92];
	char *msgptr;
	public:
	~GPS ();
    	 void initialize(void);
	void get_data(void);
	float get_lat(void);
	float get_lon(void);
	float get_prevlat(void);
	float get_prevlon(void);
	float[][] closest_waypoint(void);
};

int GPS::message_recieved(void){
	return(msg_recieved);
}

float GPS::get_lat(void){
	return latlon.lat;
}


float GPS::get_lon(void){
	return latlon.lon;
}

float GPS::get_prevlat(void){
	return latlon.prev_lat;
}


float GPS::get_prevlon(void){
	return latlon.prev_lon;
}

void GPS::open_port(void)
    {
      
    struct termios options;

    /*
     * Get the current options for the port...
     */

    tcgetattr(filedesc, &options);

    /*
     * Set the baud rates to 19200...
     */

    cfsetispeed(&options, B38400);
    cfsetospeed(&options, B38400);

    /*
     * Enable the receiver and set local mode...
     */

    options.c_cflag |= (CLOCAL | CREAD);

    /*
     * Set the new options for the port...
     */

    tcsetattr(filedesc, TCSANOW, &options);

      filedesc = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
      if (filedesc == -1)
      {
       /*
	* Could not open the port.
	*/

	perror("open_port: Unable to open /dev/ttyUSB0 - ");
      }
      else

	fcntl(filedesc, F_SETFL, 0);
   
}

void GPS::read_port(void){
	     /* Current char in buffer */

        int  nbytes;       /* Number of bytes read */
	bufptr = buffer;
	
	nbytes = read(filedesc, bufptr, 1);
	//printf("%c", buffer[0]);
}
	
	
void GPS::print(void){
	printf("Latitude:%f%c Longitude:%f%c\n",latlon.lat,latlon.lat_dir,latlon.lon,latlon.lon_dir);
		
}

void GPS::write_port(void){
	
	

	// sets baud rate to B38400 n = write(filedesc, "$PGRMC,,,,,,,,,,8,,,,\x0D\x0A",100);
 	//int n = write(filedesc, "$PGRMC,,,,,,,,,,8,,,,\x0D\x0A",100);
	 //n = write(filedesc, "$PGRMI,,,,,,,R\x0D\x0A",100);


	int n = write(filedesc, "$PGRMO,XXXXX,2,\x0D\x0A",100);
	n = write(filedesc, "$PGRMO,GPGLL,1,\x0D\x0A",100);
	n = write(filedesc, "$PGRMO,PGRME,1,\x0D\x0A",100);
    	if (n < 0)
      	fputs("write() of 4 bytes failed!\n", stderr);
 	
}

void GPS::check_message(void){
	msg_recieved = 0;
	while(msg_recieved == 0){
		read_port();
		if(buffer[0]== 0x24 ){
		 	//printf("Got $");
			msg_recieved = 1;
			int msg_map = 0;
		 	while(buffer[0]!=42){
			 	read_port();
			 	//printf("%c", buffer[0]);
				message[msg_map]=buffer[0];
				msg_map++;
			
			}
		
			parse_message();
			//printf("\n");
		 
		 }
	}
	msg_recieved = 0;
	
}

void GPS::parse_message(void){
	char * pch;
	pch = strtok (message,",");
	if(strncmp("GPGLL",pch,5)==0){
		//printf("got lat:");
		pch = strtok (NULL, ",");
		//printf ("messlat: %s\n",pch);
		latlon.lat = atof(pch);
		//printf("storelat:%f\n",latlon.lat);
		pch = strtok (NULL, ",");
		latlon.lat_dir = * pch;
		pch = strtok (NULL, ",");
		//printf ("messlon: %s\n",pch);
		latlon.lon = atof(pch);
		pch = strtok (NULL, ",");
		latlon.lon_dir = *pch;
		//printf("storelon:%f\n",latlon.lon);
		
	}
	while(pch!=NULL){
		//printf("doing some more tokens\n");
		pch = strtok (NULL, ",");
	}

}


void GPS::initialize(void){

	open_port();
	write_port();
	latlon.lat = 0;
	latlon.lon = 0;
	latlon.prev_lat = 0;
	latlon.prev_lon = 0;
	 //print_message();

}

void GPS::get_data(void){
	latlon.prev_lat = latlon.lat;
	latlon.prev_lon = latlon.lon;
	check_message();
	print();
}




GPS::~GPS () {
	close(filedesc);
}


	




