/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/*
 * $Id: sonyevid30.cc,v 1.24 2006/02/27 18:19:03 gerkey Exp $
 *
 * methods for initializing, commanding, and getting data out of
 * the Sony EVI-D30 PTZ camera
 */

/** @ingroup drivers */
/** @{ */
/** @defgroup driver_sonyevid30 sonyevid30
 * @brief Sony EVI-D30 and EVI-D100 pan-tilt-zoom cameras

The sonyevid30 driver provides control of a Sony EVI-D30 and Sony EVI-D100
pan-tilt-zoom camera units.

The sonyevid30 driver operates over a direct serial link, not
through the P2OS microcontroller's AUX port, as is the normal
configuration for ActivMedia robots.  You may have to make or buy
a cable to connect your camera to a normal serial port.  Look <a
href="http://playerstage.sourceforge.net/faq.html#evid30_wiring">here</a>
for more information and wiring instructions.

The sonyevid30 driver only supports position control.

@par Compile-time dependencies

- none

@par Provides

- @ref interface_ptz

@par Requires

- None

@par Configuration requests

- PLAYER_PTZ_GENERIC_CONFIG_REQ

@par Configuration file options

- port (string)
  - Default: "/dev/ttyS2"
  - The serial port to be used.

- fov (integer tuple)
  - Default: [3 30]
  - The minimum and maximum fields of view (in degrees), which will depend on
   the lens(es) you are using.  Half-angle??

- movement (integer)
  - Default: 0
  - Movement mode (?)
 
@par Example 

@verbatim
driver
(
  name "sonyevid30"
  provides ["ptz:0"]
  port "/dev/ttyS2"
  fov [3 30]
)
@endverbatim

@author Brian Gerkey, Brad Tonkes (D100 mode)

*/
/** @} */

#ifdef HAVE_CONFIG_H
  #include "config.h"
#endif

#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>

#include <libplayercore/playercore.h>
#include <replace/replace.h>

#define MODEL_D3X 0x0402
#define MODEL_D100 0x040D

#define PTZ_SLEEP_TIME_USEC 100000

#define MAX_PTZ_PACKET_LENGTH 16
#define MAX_PTZ_MESSAGE_LENGTH 14
#define MAX_PTZ_REPLY_LENGTH 11

#define MAX_VER_MESSAGE_LENGTH 4
#define MAX_VER_REPLY_LENGTH 14

#define PTZ_PAN_MAX 100.0
#define PTZ_TILT_MAX 25.0

#define PTZ_MAX_PAN_SPEED	0x18
#define PTZ_MAX_TILT_SPEED	0x14

#define DEFAULT_PTZ_PORT "/dev/ttyS2"

#define VISCA_COMMAND_CODE	0x01
#define VISCA_INQUIRY_CODE	0x09

class SonyEVID30:public Driver 
{
 protected:
  bool command_pending1;  // keep track of how many commands are pending;
  bool command_pending2;  // that way, we can cancel them if necessary
  bool ptz_fd_blocking;
  
  // internal methods
  int Send(unsigned char* str, int len, unsigned char* reply, uint8_t camera = 1);
  int Receive(unsigned char* reply);
  int SendCommand(unsigned char* str, int len, uint8_t camera = 1);
  int CancelCommand(char socket);
  int SendRequest(unsigned char* str, int len, unsigned char* reply, uint8_t camera = 1);
//  int HandleConfig(void *client, unsigned char *buf, size_t len);

  // MessageHandler
  int ProcessMessage(MessageQueue* resp_queue, player_msghdr * hdr, void * data);

  // this function will be run in a separate thread
  virtual void Main();

  virtual int GetCameraType(int *model);

  virtual int SendAbsPanTilt(short pan, short tilt);
  virtual int SendStepPan(int);
  virtual int SendStepTilt(int);
  virtual int SendAbsZoom(short zoom);
  virtual int GetAbsZoom(short* zoom);
  virtual int GetAbsPanTilt(short* pan, short* tilt);
  virtual void PrintPacket(char* str, unsigned char* cmd, int len);

  double ptz_pan_conv_factor;
  double ptz_tilt_conv_factor;

 public:
  int ptz_fd; // ptz device file descriptor
  /* device used to communicate with the ptz */
  char ptz_serial_port[MAX_FILENAME_SIZE];

  // Min and max values for camera field of view (degrees).
  // These are used to compute appropriate zoom values.
  int maxfov, minfov;

protected:
  struct pollfd read_pfd;

  int movement_mode;
  int pandemand;
  int tiltdemand;

public:

  SonyEVID30( ConfigFile* cf, int section);

  virtual int Setup();
  virtual int Shutdown();
};
  
// initialization function
Driver* SonyEVID30_Init( ConfigFile* cf, int section)
{
  return((Driver*)(new SonyEVID30( cf, section)));
}

/* how to make this work for multiple cameras...
   want to make a player device for each camera, ie ptz:0 ptz:1, so can read/write commands
   on the client side independently of how they are controlled.

   but for the sonys, sets of cameras are paritioned by serial port.  so
   we add a parameter "camera" to the config for ptz, and then here we have a table
   which keeps track of instantiations of devices according to serial port.
   
   will need to redo the class so that cameras on the same serial port share the port
   instead of each trying to open it.  they also have a port-id.  
   
   so _Init will read the config file and based on the serial port and camera parameter
   it will either create an instance of a serial-owning device, or instantiate a camera
   that shares an existing port.

   so SonyEVIController is the one that controls the port
   and create SonyEVIPeripheral that are the cameras.  each peripheral has an id that
   is given to create packets for that peripheral.  

   problem is this makes broadcasting commands more difficult/less efficient.
   use the new Wait and GetAvailable to share the port...
*/

// a driver registration function
void 
SonyEVID30_Register(DriverTable* table)
{
  table->AddDriver("sonyevid30",  SonyEVID30_Init);
}

SonyEVID30::SonyEVID30( ConfigFile* cf, int section) 
: Driver(cf, section, false, PLAYER_MSGQUEUE_DEFAULT_MAXLEN, PLAYER_PTZ_CODE)
{
  ptz_fd = -1;
  command_pending1 = false;
  command_pending2 = false;

  movement_mode = 0;
  pandemand = 0;
  tiltdemand=0;

  read_pfd.events = POLLIN;

  // TODO: check field of view values.
  this->minfov = (int) RTOD(cf->ReadTupleAngle(section, "fov", 0, DTOR(3)));
  this->maxfov = (int) RTOD(cf->ReadTupleAngle(section, "fov", 1, DTOR(30)));

  // Assume we've got a D3X
  this->ptz_pan_conv_factor = 0x0370 / (double) PTZ_PAN_MAX;
  this->ptz_tilt_conv_factor = 0x012C / (double) PTZ_TILT_MAX;

  movement_mode = (int) cf->ReadInt(section, "movement", 0);

  strncpy(ptz_serial_port,
          cf->ReadString(section, "port", DEFAULT_PTZ_PORT),
          sizeof(ptz_serial_port));
}

int 
SonyEVID30::Setup()
{
  struct termios term;
  short pan,tilt;
  int cam_type;
  int flags;

  printf("PTZ connection initializing (%s)...", ptz_serial_port);
  fflush(stdout);

  // open it.  non-blocking at first, in case there's no ptz unit.
  if((ptz_fd = open(ptz_serial_port, O_RDWR | O_SYNC | O_NONBLOCK, S_IRUSR | S_IWUSR )) < 0 )
  {
    perror("SonyEVID30::Setup():open():");
    return(-1);
  }  

  read_pfd.fd = ptz_fd;

  if(tcflush(ptz_fd, TCIFLUSH ) < 0 )
  {
    perror("SonyEVID30::Setup():tcflush():");
    close(ptz_fd);
    ptz_fd = -1;
    return(-1);
  }
  if(tcgetattr(ptz_fd, &term) < 0 )
  {
    perror("SonyEVID30::Setup():tcgetattr():");
    close(ptz_fd);
    ptz_fd = -1;
    return(-1);
  }
  
  cfmakeraw(&term);
  cfsetispeed(&term, B9600);
  cfsetospeed(&term, B9600);
  
  if(tcsetattr(ptz_fd, TCSAFLUSH, &term) < 0 )
  {
    perror("SonyEVID30::Setup():tcsetattr():");
    close(ptz_fd);
    ptz_fd = -1;
    return(-1);
  }

	/* Work out what version of camera we are: d3x or d100.  The parameters of
	 * each are slightly different.
	 */
	if (GetCameraType(&cam_type)) {
		printf("Couldn't connect to PTZ device most likely because the camera\n"
				"is not connected or is connected not to %s\n", 
				ptz_serial_port);
		close(ptz_fd);
		ptz_fd = -1;
		return -1;
	} else {
		/* Here we calculate our conversion factors.
		 * The D3X series has a tilt range of +/- 25.0 degrees corresponding
		 * with +/- 0x12C, and a pan range of +/- 100.0 degrees corresponding
		 * with +/- 0x370.
		 * The D100 series has the same physical pan and tilt ranges, but
		 * represents these as +/- 0x168 (tilt) and 0x5A0 (pan).
		 */
		switch (cam_type) {
		case MODEL_D3X:
			this->ptz_pan_conv_factor = 0x0370 / (double) PTZ_PAN_MAX;
			this->ptz_tilt_conv_factor = 0x012C / (double) PTZ_TILT_MAX;
			break;
		case MODEL_D100:
			this->ptz_pan_conv_factor = 0x05A0 / (double) PTZ_PAN_MAX;
			this->ptz_tilt_conv_factor = 0x0168 / (double) PTZ_TILT_MAX;
			break;
		default:
			printf("Unknown camera type: %d\n", cam_type);
			break;
		}
	}

  ptz_fd_blocking = false;
  /* try to get current state, just to make sure we actually have a camera */
  if(GetAbsPanTilt(&pan,&tilt))
  {
    printf("Couldn't connect to PTZ device most likely because the camera\n"
                    "is not connected or is connected not to %s\n", 
                    ptz_serial_port);
    close(ptz_fd);
    ptz_fd = -1;
    return(-1);
  }

  /* ok, we got data, so now set NONBLOCK, and continue */
  if((flags = fcntl(ptz_fd, F_GETFL)) < 0)
  {
    perror("SonyEVID30::Setup():fcntl()");
    close(ptz_fd);
    ptz_fd = -1;
    return(1);
  }
  if(fcntl(ptz_fd, F_SETFL, flags ^ O_NONBLOCK) < 0)
  {
    perror("SonyEVID30::Setup():fcntl()");
    close(ptz_fd);
    ptz_fd = -1;
    return(1);
  }
  ptz_fd_blocking = true;
  puts("Done.");

  // zero the command and data buffers
//  player_ptz_data_t data;
//  player_ptz_cmd_t cmd;

//  data.pan = data.tilt = data.zoom = 0;
//  cmd.pan = cmd.tilt = 0;
//  cmd.zoom = this->maxfov;

//  PutData((void*)&data,sizeof(data),NULL);
//  PutCommand(this->device_id,(void*)&cmd,sizeof(cmd),NULL);

  // start the thread to talk with the camera
  StartThread();

  return(0);
}

int 
SonyEVID30::Shutdown()
{
  puts("SonyEVID30::Shutdown");

  if(ptz_fd == -1)
    return(0);

  StopThread();

  // put the camera back to center
  usleep(PTZ_SLEEP_TIME_USEC);
  SendAbsPanTilt(0,0);
  usleep(PTZ_SLEEP_TIME_USEC);
  SendAbsZoom(0);

  if(close(ptz_fd))
    perror("SonyEVID30::Shutdown():close():");
  ptz_fd = -1;
  puts("PTZ camera has been shutdown");
  return(0);
}

int
SonyEVID30::Send(unsigned char* str, int len, unsigned char* reply, uint8_t camera)
{
  unsigned char command[MAX_PTZ_PACKET_LENGTH];
  int i;

  if(len > MAX_PTZ_MESSAGE_LENGTH)
  {
    fprintf(stderr, "SonyEVID30::Send(): message is too large (%d bytes)\n",
                    len);
    return(-1);
  }

  assert(camera < 8);

  command[0] = 0x80 | camera; // controller address is 0, camera address 1
  for(i=0;i<len;i++)
    command[i+1] = str[i];

  command[i+1] = 0xFF;  // packet terminator

  //PrintPacket("Sending", command, i+2);
  
  // send the command
  if(write(ptz_fd, command, i+2) < 0)
  {
    perror("SonyEVID30::Send():write():");
    return(-1);
  }

  //puts("Send(): calling Receive()");
  return(Receive(reply));
}

int
SonyEVID30::Receive(unsigned char* reply)
{
  static unsigned char buffer[MAX_PTZ_PACKET_LENGTH];
  static int numread = 0;

  unsigned char temp_reply[MAX_PTZ_PACKET_LENGTH];
  int newnumread = 0;
  int bufptr = -1;
  int i;
  int temp;
  int pret;
  // if we're non-blocking, then we should wait a bit to give the
  // camera a chance to respond. 
  //  if(!ptz_fd_blocking)
  //    usleep(PTZ_SLEEP_TIME_USEC);


  
  memset(temp_reply,0,MAX_PTZ_PACKET_LENGTH);
  memset(reply,0,MAX_PTZ_PACKET_LENGTH);
  if(numread > 0)
  {
    //printf("copying %d old bytes\n", numread);
    memcpy(temp_reply,buffer,numread);
    // look for the terminator
    for(i=0;i<numread;i++)
    {
      if(temp_reply[i] == 0xFF)
      {
        bufptr = i;
        break;
      }
    }
  }

  while(bufptr < 0)
    {
      pret = poll(&read_pfd, 1, 1000);
      if (pret == 0) {
	printf("SONY: poll timedout !\n");
      } else if (pret < 0) {
	printf("SONY: poll returned error!\n");
      }
    newnumread = read(ptz_fd, temp_reply+numread, MAX_PTZ_REPLY_LENGTH-numread);
    if((numread += newnumread) < 0)
    {
      perror("SonyEVID30::Send():read():");
      return(-1);
    }
    else if(!newnumread)
    {
      // hmm...we were expecting something, yet we read
      // zero bytes. some glitch.  drain input, and return
      // zero.  we'll get a message next time through
      //puts("Receive(): read() returned 0");
      if(tcflush(ptz_fd, TCIFLUSH ) < 0 )
      {
        perror("SonyEVID30::Send():tcflush():");
        return(-1);
      }
      numread = 0;
      return(0);
    }
    // look for the terminator
    for(i=0;i<numread;i++)
    {
      if(temp_reply[i] == 0xFF)
      {
        bufptr = i;
        break;
      }
    }
  }

  temp = numread;
  // if we read extra bytes, keep them around
  if(bufptr == numread-1)
    numread = 0;
  else
  {
    //printf("storing %d bytes\n", numread-(bufptr+1));
    memcpy(buffer,temp_reply+bufptr+1,numread-(bufptr+1));
    numread = numread-(bufptr+1);
  }

  //PrintPacket("Really Received", temp_reply, temp);
  //PrintPacket("Received", temp_reply, bufptr+1);
  
  // strip off leading trash, up to start character 0x90
  for(i = 0;i< bufptr;i++)
  {
    if(temp_reply[i] == 0x90 && temp_reply[i+1] != 0x90)
      break;
  }
  //if(i)
    //printf("SonyEVID30::Receive(): strip off zeros up to: %d\n", i);
  if(i == bufptr)
    return(0);
  memcpy(reply,temp_reply+i,bufptr+1-i);

  // if it's a command completion, record it, then go again
  if((reply[0] == 0x90) && ((reply[1] >> 4) == 0x05) && (reply[2] == 0xFF))
  {
    //puts("got command completion");
    if((reply[1] & 0x0F) == 0x01)
      command_pending1 = false;
    else if((reply[1] & 0x0F) == 0x02)
      command_pending2 = false;
  }

  return(bufptr+1-i);
}

int
SonyEVID30::CancelCommand(char socket)
{
  unsigned char command[MAX_PTZ_MESSAGE_LENGTH];
  unsigned char reply[MAX_PTZ_MESSAGE_LENGTH];
  int reply_len;
  
  //printf("Canceling socket %d\n", socket);

  command[0] = socket;
  command[0] |= 0x20;
  
  if((reply_len = Send(command, 1, reply)) <= 0)
    return(reply_len);
  
  // wait for the response
  while((reply[0] != 0x90) || ((reply[1] >> 4) != 0x06) || 
        !((reply[2] == 0x04) || (reply[2] == 0x05)) || (reply_len != 4))
  {
    if((reply[0] != 0x90) || ((reply[1] >> 4) != 0x05) || (reply[2] != 0xFF))
      PrintPacket("SonyEVID30::CancelCommand(): unexpected response",reply,
                      reply_len);
    //puts("CancelCommand(): calling Receive()");
    if((reply_len = Receive(reply)) <= 0)
      return(reply_len);
  }
  if(socket == 1)
    command_pending1 = false;
  else if(socket == 2)
    command_pending2 = false;
  return(0);
}

int 
SonyEVID30::SendCommand(unsigned char* str, int len, uint8_t camera)
{
  unsigned char reply[MAX_PTZ_PACKET_LENGTH];
  int reply_len;

  if(command_pending1 && command_pending2)
  {
    if((command_pending1 && CancelCommand(1)) || 
       (command_pending2 && CancelCommand(2)))
      return(-1);
  }

  if(command_pending1 && command_pending2)
  {
    puts("2 commands still pending. wait");
    return(-1);
  }

  

  if((reply_len = Send(str, len, reply)) <= 0)
    return(reply_len);
  
  // wait for the ACK
  while((reply[0] != 0x90) || ((reply[1] >> 4) != 0x04) || (reply_len != 3))
  {
    if((reply[0] != 0x90) || ((reply[1] >> 4) != 0x05) || (reply_len != 3))
    {
      PrintPacket("SonyEVID30::SendCommand(): expected ACK, but got", 
                      reply,reply_len);
    }
    //puts("SendCommand(): calling Receive()");
    if((reply_len = Receive(reply)) <= 0)
      return(reply_len);
  }
  
  if((reply[1] & 0x0F) == 0x01)
    command_pending1 = true;
  else if((reply[1] & 0x0F) == 0x02)
    command_pending2 = true;
  else
    fprintf(stderr,"SonyEVID30::SendCommand():got ACK for socket %d\n",
                    reply[1] & 0x0F);

  return(0);
}


int 
SonyEVID30::SendRequest(unsigned char* str, int len, unsigned char* reply, uint8_t camera)
{
  int reply_len;

  if((reply_len = Send(str, len, reply, camera)) <= 0)
    return(reply_len);
  
  // check that it's an information return
  while((reply[0] != 0x90) || (reply[1] != 0x50))
  {
    if((reply[0] != 0x90) || ((reply[1] >> 4) != 0x05) || (reply_len != 3))
    {
      PrintPacket("SonyEVID30::SendCommand(): expected information return, but got", 
                    reply,reply_len);
    }
    //puts("SendRequest(): calling Receive()");
    if((reply_len = Receive(reply)) <= 0)
      return(reply_len);
  }

  return(reply_len);
}

int
SonyEVID30::SendAbsPanTilt(short pan, short tilt)
{
  unsigned char command[MAX_PTZ_MESSAGE_LENGTH];
  short convpan,convtilt;

  printf("Send abs pan/tilt: %d, %d\n", pan, tilt);

  if (abs(pan)>(short)PTZ_PAN_MAX) 
  {
    if (pan < (short) -PTZ_PAN_MAX)
      pan = (short)-PTZ_PAN_MAX;
    else if (pan > (short) PTZ_PAN_MAX)
      pan = (short)PTZ_PAN_MAX;
    puts("Camera pan angle thresholded");
  }

  if(abs(tilt)>(short)PTZ_TILT_MAX) 
  {
    if(tilt<(short)-PTZ_TILT_MAX)
      tilt=(short)-PTZ_TILT_MAX;
    else if(tilt>(short)PTZ_TILT_MAX)
      tilt=(short)PTZ_TILT_MAX;
    puts("Camera tilt angle thresholded");
  }

  convpan = (short)(pan*ptz_pan_conv_factor);
  convtilt = (short)(tilt*ptz_tilt_conv_factor);

  printf("[Conv] Send abs pan/tilt: %d, %d\n", convpan, convtilt);

  command[0] = 0x01;  // absolute position command
  command[1] = 0x06;  // absolute position command
  command[2] = 0x02;  // absolute position command
  command[3] = PTZ_MAX_PAN_SPEED;  // MAX pan speed
  command[4] = PTZ_MAX_TILT_SPEED;  // MAX tilt speed
  // pan position
  command[5] =  (unsigned char)((convpan & 0xF000) >> 12); 
  command[6] = (unsigned char)((convpan & 0x0F00) >> 8);
  command[7] = (unsigned char)((convpan & 0x00F0) >> 4);
  command[8] = (unsigned char)(convpan & 0x000F); 
  // tilt position
  command[9] = (unsigned char)((convtilt & 0xF000) >> 12); 
  command[10] = (unsigned char)((convtilt & 0x0F00) >> 8);
  command[11] = (unsigned char)((convtilt & 0x00F0) >> 4);
  command[12] = (unsigned char)(convtilt & 0x000F); 

  return(SendCommand(command, 13));
}

int
SonyEVID30::SendStepPan(int dir)
{
  unsigned char cmd[MAX_PTZ_MESSAGE_LENGTH];
  
  cmd[0] = 0x01;
  cmd[1] = 0x06;
  cmd[2] = 0x01;
  cmd[3] = PTZ_MAX_PAN_SPEED;
  cmd[4] = PTZ_MAX_TILT_SPEED;
  // if dir >= 0 then pan left, else right
  cmd[5] = dir >= 0 ? 0x01 : 0x02;
  cmd[6] = 0x03;

  printf("step pan\n");

  return (SendCommand(cmd, 7));
}

int
SonyEVID30::SendStepTilt(int dir)
{
  unsigned char cmd[MAX_PTZ_MESSAGE_LENGTH];
  
  cmd[0] = 0x01;
  cmd[1] = 0x06;
  cmd[2] = 0x01;
  cmd[3] = PTZ_MAX_PAN_SPEED;
  cmd[4] = PTZ_MAX_TILT_SPEED;
  cmd[5] = 0x03;
  // if dir >= 0 then tilt up, else down
  cmd[6] = dir >= 0 ? 0x01 : 0x02;

  return (SendCommand(cmd, 7));
}

int
SonyEVID30::GetAbsPanTilt(short* pan, short* tilt)
{
  unsigned char command[MAX_PTZ_MESSAGE_LENGTH];
  unsigned char reply[MAX_PTZ_PACKET_LENGTH];
  int reply_len;
  short convpan, convtilt;

  command[0] = 0x09;
  command[1] = 0x06;
  command[2] = 0x12;
  
  if((reply_len = SendRequest(command,3,reply)) <= 0)
    return(reply_len);

  // first two bytes are header (0x90 0x50)
  
  // next 4 are pan
  convpan = reply[5];
  convpan |= (reply[4] << 4);
  convpan |= (reply[3] << 8);
  convpan |= (reply[2] << 12);

  *pan = (short)(convpan / ptz_pan_conv_factor);
  
  // next 4 are tilt
  convtilt = reply[9];
  convtilt |= (reply[8] << 4);
  convtilt |= (reply[7] << 8);
  convtilt |= (reply[6] << 12);

  *tilt = (short)(convtilt / ptz_tilt_conv_factor);

  return(0);
}

int
SonyEVID30::GetAbsZoom(short* zoom)
{
  unsigned char command[MAX_PTZ_MESSAGE_LENGTH];
  unsigned char reply[MAX_PTZ_PACKET_LENGTH];
  int reply_len;

  command[0] = 0x09;
  command[1] = 0x04;
  command[2] = 0x47;

  if((reply_len = SendRequest(command,3,reply)) <= 0)
    return(reply_len);

  // first two bytes are header (0x90 0x50)
  // next 4 are zoom
  *zoom = reply[5];
  *zoom |= (reply[4] << 4);
  *zoom |= (reply[3] << 8);
  *zoom |= (reply[2] << 12);

  return(0);
}

/*
* Get the device type and version information from the camera.  Query packet is
* in the format: 8x 09 00 02 FF, return packet in the format: y0 50 gg gg hh hh
* jj jj kk FF where
* 	gggg: Vendor ID
*   hhhh: Model ID
*   jjjj: ROM Version
*   kk: socket number (2)
*/
int 
SonyEVID30::GetCameraType(int *model)
{
	unsigned char command[MAX_VER_MESSAGE_LENGTH];
	unsigned char reply[MAX_VER_REPLY_LENGTH];
	int reply_len;

	command[0] = 0x09;
	command[1] = 0x00;
	command[2] = 0x02;

	if ((reply_len = SendRequest(command, 3, reply)) <= 0) {
		return reply_len;
	} else {
		*model = (reply[4] << 8) + reply[5];
	}
	return 0;
}

int
SonyEVID30::SendAbsZoom(short zoom)
{
  unsigned char command[MAX_PTZ_MESSAGE_LENGTH];

  if(zoom<0) {
    zoom=0;
    //puts("Camera zoom thresholded");
  }
  else if(zoom>1023){
    zoom=1023;
    //puts("Camera zoom thresholded");
  }

  command[0] = 0x01;  // absolute position command
  command[1] = 0x04;  // absolute position command
  command[2] = 0x47;  // absolute position command
  
  // zoom position
  command[3] =  (unsigned char)((zoom & 0xF000) >> 12); 
  command[4] = (unsigned char)((zoom & 0x0F00) >> 8);
  command[5] = (unsigned char)((zoom & 0x00F0) >> 4);
  command[6] = (unsigned char)(zoom & 0x000F); 

  return(SendCommand(command, 7));
}


int SonyEVID30::ProcessMessage(MessageQueue * resp_queue, player_msghdr * hdr, void * data)
{
  assert(hdr);
  assert(data);

  if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ, PLAYER_PTZ_REQ_GENERIC, device_addr))
  {
    assert(hdr->size == sizeof(player_ptz_req_generic_t));

    player_ptz_req_generic_t *cfg = (player_ptz_req_generic_t *)data;

    // check whether command or inquiry...
    if (cfg->config[0] == VISCA_COMMAND_CODE) 
    {
      if (SendCommand((uint8_t *)cfg->config, cfg->config_count) < 0) 
        Publish(device_addr, resp_queue, PLAYER_MSGTYPE_RESP_NACK, hdr->subtype);
      else
        Publish(device_addr, resp_queue, PLAYER_MSGTYPE_RESP_ACK, hdr->subtype);
      return 0;
    } 
    else 
    {
      // this is an inquiry, so we have to send data back
      cfg->config_count = SendRequest((uint8_t*)cfg->config, cfg->config_count, (uint8_t*)cfg->config);
      Publish(device_addr, resp_queue, PLAYER_MSGTYPE_RESP_ACK, hdr->subtype);
    }
  }

  if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_CMD, PLAYER_PTZ_CMD_STATE, device_addr))
  {
    short zoomdemand=0;
    bool newpantilt=true, newzoom=true;
  	
  	assert (hdr->size == sizeof (player_ptz_cmd_t));
  	player_ptz_cmd_t command = *reinterpret_cast<player_ptz_cmd_t *> (data);
    if(pandemand != (int)rint(RTOD(command.pan)))
    {
      pandemand = (int)rint(RTOD(command.pan));
      newpantilt = true;
    }
    if(tiltdemand != (int)rint(RTOD(command.tilt)))
    {
      tiltdemand = (int)rint(RTOD(command.tilt));
      newpantilt = true;
    }
    
    zoomdemand = (1024 * (zoomdemand - this->maxfov)) / (this->minfov - this->maxfov);
    
    if(newzoom)
    {
	  if(SendAbsZoom(zoomdemand))
	  {
	    fputs("SonyEVID30:Main():SendAbsZoom() errored. bailing.\n", stderr);
	    pthread_exit(NULL);
	  }
    }
    
    if(newpantilt)
    {
	  if (!movement_mode) 
	  {
	    pandemand = -pandemand;
	    if(SendAbsPanTilt(pandemand,tiltdemand))
	    {
	      fputs("SonyEVID30:Main():SendAbsPanTilt() errored. bailing.\n", stderr);
	      pthread_exit(NULL);
	    }
	  }
    } 
	return 0;
  }

  return -1;
}


void
SonyEVID30::PrintPacket(char* str, unsigned char* cmd, int len)
{
  printf("%s: ", str);
  for(int i=0;i<len;i++)
    printf(" %.2x", cmd[i]);
  puts("");
}

// this function will be run in a separate thread
void 
SonyEVID30::Main()
{
  player_ptz_data_t data;
//  char buffer[256];
//  size_t buffer_len;
//  void *client;
  short pan,tilt,zoom;
  
  while(1) 
  {
    pthread_testcancel();
    ProcessMessages();
    pthread_testcancel();
    
    
    /* get current state */
    if(GetAbsPanTilt(&pan,&tilt))
    {
	  fputs("SonyEVID30:Main():GetAbsPanTilt() errored. bailing.\n", stderr);
	  pthread_exit(NULL);
    }
    /* get current state */
    if(GetAbsZoom(&zoom))
    {
	  fputs("SonyEVID30:Main():GetAbsZoom() errored. bailing.\n", stderr);
	  pthread_exit(NULL);
    }
    
    // Do the necessary coordinate conversions.  Camera's natural pan
    // coordinates increase clockwise; we want them the other way, so
    // we negate pan here.  Zoom values are converted from arbitrary
    // units to a field of view (in degrees).
    pan = -pan;
    zoom = this->maxfov + (zoom * (this->minfov - this->maxfov)) / 1024; 
    
    if (movement_mode) 
	{
	  if (pandemand-pan) 
	  {
	    SendStepPan(pandemand-pan);
	  }
	
	  if (tiltdemand - tilt) 
	  {
	    SendStepTilt(tiltdemand - tilt);
	  }
	}
  
    // Copy the data.
    data.pan = DTOR(pan);
    data.tilt = DTOR(tilt);
    data.zoom = DTOR(zoom);
    
    /* test if we are supposed to cancel */
    pthread_testcancel();
    Publish(device_addr, NULL, PLAYER_MSGTYPE_DATA, PLAYER_PTZ_DATA_STATE, &data,sizeof(player_ptz_data_t),NULL);
       
    usleep(PTZ_SLEEP_TIME_USEC);
    }
}

