#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <stdlib.h>
#include <fcntl.h>
#include <termios.h>

#include <replace/replace.h>

#include "urg_laser.h"

int 
urg_laser::ReadUntil(int fd, unsigned char *buf, int len, int timeout)
{
  int ret;
  int current=0;
  struct pollfd ufd[1];
  int retval;

  ufd[0].fd = fd;
  ufd[0].events = POLLIN;

  do
  {
    if(timeout >= 0)
    {
      if((retval = poll(ufd,1,timeout)) < 0)
      {
        perror("poll():");
        return(-1);
      }
      else if(retval == 0)
      {
        puts("Timed out on read");
        return(-1);
      }
    }

    ret = read(fd,&buf[current],len-current);
    if (ret < 0)
      return ret;
    current += ret;
    if (current > 2 && buf[current-2] == '\n' && buf[current-1] == '\n')
    {
      puts("Got an end of command while waiting for more data, this is bad\n");
      return -1;	
    }
  } while(current < len);
  return len;
}

urg_laser::urg_laser()
{
	laser_port = NULL;
}

urg_laser::~urg_laser()
{
	if (PortOpen())
		fclose(laser_port);
}

int 
urg_laser::ChangeBaud(int curr_baud, int new_baud, int timeout)
{
  struct termios newtio;
  int fd;

  fd = fileno(laser_port);

  if(tcgetattr(fd, &newtio) < 0)
  {
    perror("urg_laser::ChangeBaud:tcgetattr():");
    close(fd);
    return(-1);
  }
  cfmakeraw(&newtio);
  cfsetispeed(&newtio, curr_baud);
  cfsetospeed(&newtio, curr_baud);

  if(tcsetattr(fd, TCSAFLUSH, &newtio) < 0 )
  {
    perror("urg_laser::ChangeBaud:tcsetattr():");
    close(fd);
    return(-1);
  }

  unsigned char buf[17];
  memset(buf,0,sizeof(buf));
  buf[0] = 'S';
  switch(new_baud)
  {
    case B19200:
      buf[1] = '0';
      buf[2] = '1';
      buf[3] = '9';
      buf[4] = '2';
      buf[5] = '0';
      buf[6] = '0';
      break;
    case B57600:
      buf[1] = '0';
      buf[2] = '5';
      buf[3] = '7';
      buf[4] = '6';
      buf[5] = '0';
      buf[6] = '0';
      break;
    case B115200:
      buf[1] = '1';
      buf[2] = '1';
      buf[3] = '5';
      buf[4] = '2';
      buf[5] = '0';
      buf[6] = '0';
      break;
    default:
      printf("unknown baud rate %d\n", new_baud);
      return(-1);
  }
  buf[7] = '0';
  buf[8] = '0';
  buf[9] = '0';
  buf[10] = '0';
  buf[11] = '0';
  buf[12] = '0';
  buf[13] = '0';
  buf[14] = '\n';

  fprintf(laser_port,"%s",buf);
  memset(buf,0,sizeof(buf));
  int len;
  // The docs say that the response ends in 'status LF LF', where
  // status is '0' if everything went alright.  But it seems that
  // the response actually ends in 'LF status LF'.
  if(((len = ReadUntil(fd,buf,sizeof(buf),timeout)) < 0) ||
     (buf[15] != '0'))
  {
    puts("failed to change baud rate");
    return(-1);
  }
  else
  {
    if(tcgetattr(fd, &newtio) < 0)
    {
      perror("urg_laser::ChangeBaud:tcgetattr():");
      close(fd);
      return(-1);
    }
    cfmakeraw(&newtio);
    cfsetispeed(&newtio, new_baud);
    cfsetospeed(&newtio, new_baud);
    if(tcsetattr(fd, TCSAFLUSH, &newtio) < 0 )
    {
      perror("urg_laser::ChangeBaud:tcsetattr():");
      close(fd);
      return(-1);
    }
    else
    {
      usleep(200000);
      return(0);
    }
  }
}

int 
urg_laser::Open(const char * PortName, int use_serial, int baud)
{
  if(PortOpen())
    this->Close();

  laser_port = fopen(PortName,"r+");
  if (laser_port == NULL)
  {
    printf("Failed to open Port: %s error = %d:%s\n",PortName,errno,strerror(errno));
    return(-1);
  }

  int fd = fileno(laser_port);

  if(use_serial)
  {
    puts("Trying to connect at 19200");
    if(this->ChangeBaud(B19200, baud, 100) != 0)
    {
      puts("Trying to connect at 57600");
      if(this->ChangeBaud(B57600, baud, 100) != 0)
      {
        puts("Trying to connect at 115200");
        if(this->ChangeBaud(B115200, baud, 100) != 0)
        {
          puts("failed to connect at any baud");
          close(fd);
          return(-1);
        }
      }
    }
    puts("Successfully changed baud rate");
  }
  else
  {
    // set up new settings
    struct termios newtio;
    memset(&newtio, 0,sizeof(newtio));
    newtio.c_cflag = /*(rate & CBAUD) |*/ CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;
    newtio.c_lflag = ICANON;

    // activate new settings
    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &newtio);
    usleep(200000);
  }

  return(0);
}

int
urg_laser::Close()
{
  int retval;

  assert(this->laser_port);

  retval = fclose(this->laser_port);
  this->laser_port = NULL;
  return(retval);
}


bool urg_laser::PortOpen()
{
  return laser_port != NULL;
}


int 
urg_laser::GetReadings(urg_laser_readings_t * readings)
{
  unsigned char Buffer[11];
  memset(Buffer,0,11);
  assert(readings);

  if (!PortOpen())
    return -3;

  tcflush(fileno(laser_port), TCIFLUSH);
  // send the command
  fprintf(laser_port,"G00076801\n");

  int file = fileno(laser_port);

  // check the returned command
  ReadUntil(file, Buffer,10, -1);

  /*	fscanf(laser_port,"%9s",Buffer);*/
  //Buffer[10]='\0';
  //printf("Read: %s",Buffer);

  if (strncmp((const char *) Buffer,"G00076801",9))
  {
    printf("Error reading command result: %s\n",Buffer);
    tcflush(fileno(laser_port), TCIFLUSH);
    return -1;
  }

  // check the returned status
  ReadUntil(file, Buffer,2, -1);
  //Buffer[2]='\0';
  //printf("Read: %s",Buffer);
  if (Buffer[0] != '0')
    return Buffer[0] - '0';

  for (int i=0; ; ++i)
  {
    ReadUntil(file, Buffer,2, -1);

    if (Buffer[0] == '\n' && Buffer[1] == '\n')
      break;
    else if (Buffer[0] == '\n')
    {
      Buffer[0] = Buffer[1];
      if (ReadUntil(file, &Buffer[1],1,-1) < 0)
        return -1;
    }

    if (i < MAX_READINGS)
    {	
      readings->Readings[i] = ((Buffer[0]-0x30) << 6) | (Buffer[1]-0x30);
    }
    else
    {
      printf("Got too many readings! %d\n",i);
    }
  }

  return 0;
}
