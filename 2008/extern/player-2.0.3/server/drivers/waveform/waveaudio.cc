/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  Audiodevice attempted written by Esben Ostergaard
 *
 * This program is free software; you can redistribute it and/or modify
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

/** @ingroup drivers */
/** @{ */
/** @defgroup driver_waveaudio waveaudio
 * @brief Raw audio waveforms

@todo This driver is currently disabled because it needs to be updated to
the Player 2.0 API.

The waveaudio driver captures audio from /dev/dsp on systems which
support the OSS sound driver. The data is exported using the @ref
interface_waveform generic sample data interface. Currently data
is captured at 8bit, mono, 16KHz.

@par Compile-time dependencies

- &lt;sys/soundcard.h&gt;

@par Provides

- @ref interface_waveform

@par Requires

- None

@par Configuration requests

- none

@par Configuration file options

- none
 
@par Example 

@verbatim
driver
(
  name "waveaudio"
  provides ["waveform:0"]
)
@endverbatim

@author Richard Vaughan

*/
/** @} */

#include <string.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <netinet/in.h>  /* for struct sockaddr_in, htons(3) */

#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/soundcard.h>
#include <stdio.h>
#include <math.h>
#include <sys/time.h> 
#include <unistd.h>

#include <pthread.h>

#include <error.h>
#include <driver.h>
#include <drivertable.h>

// If set, we generate tones instead of sampling from the device. This
// allows testing of this driver and a client on the same machine, as
// only one process can open the DSP at a time.
//#define TEST_TONE

const double DURATION = 0.1; /* seconds of sampling */
const int RATE = 16000; /* samples per second */
const int CHANNELS = 1; /* 1 = mono 2 = stereo */
const int DEPTH = 8; /* bits, 8 or 16 */

const int SAMPLES = (int)(DURATION*RATE*CHANNELS);
const int BYTES = SAMPLES*(DEPTH/8);

#define DEVICE "/dev/dsp"


class Waveaudio:public Driver 
{
private:
  
  int configureDSP();
  void openDSPforRead();
  
  int fd;       /* sound device file descriptor */
  
public:
  
  Waveaudio( ConfigFile* cf, int section);
  
  virtual void Main();
  virtual int Setup();
  virtual int Shutdown();
};




Driver* Waveaudio_Init( ConfigFile* cf, int section)
{
  return((Driver*)(new Waveaudio( cf, section)));
}

// a driver registration function
void 
Waveaudio_Register(DriverTable* table)
{
  table->AddDriver("wave_audio",  Waveaudio_Init);
}

Waveaudio::Waveaudio( ConfigFile* cf, int section) 
  : Driver(cf, section, true, PLAYER_MSGQUEUE_DEFAULT_MAXLEN, PLAYER_WAVEFORM_CODE, PLAYER_ALL_MODE)
{
}

int 
Waveaudio::configureDSP() 
{
#ifdef TEST_TONE
  return 0;
#else

  int arg;      /* argument for ioctl calls */
  int status;   /* return status of system calls */
  int r=0;
  
  openDSPforRead();

  /* set sampling parameters */
  arg = DEPTH;      /* sample size */
  status = ioctl(fd, SOUND_PCM_WRITE_BITS, &arg);
  if (status == -1) {
    perror("SOUND_PCM_WRITE_BITS ioctl failed"); 
    r=-1;
  }
  if (arg != DEPTH) {
    printf("SOUND_PCM_WRITE_BITS: asked for %d, got %d\n", DEPTH, arg);
    //perror("unable to set sample size");
    //r=-1;
  }
  arg = CHANNELS;  /* mono or stereo */
  status = ioctl(fd, SOUND_PCM_WRITE_CHANNELS, &arg);
  if (status == -1) {
    perror("SOUND_PCM_WRITE_CHANNELS ioctl failed");
    r=-1;
  }
  if (arg != CHANNELS) {
    perror("unable to set number of channels");
    r=-1;
  }
  arg = RATE;      /* sampling rate */
  status = ioctl(fd, SOUND_PCM_WRITE_RATE, &arg);
  if (status == -1) {
    perror("SOUND_PCM_WRITE_WRITE ioctl failed");return 0;
    r=-1;
  }

  /* set sampling parameters */
  arg = CHANNELS;  /* mono or stereo */
  status = ioctl(fd, SOUND_PCM_READ_CHANNELS, &arg);
  if (status == -1) {
    perror("SOUND_PCM_READ_CHANNELS ioctl failed");
    r=-1;
  }
  if (arg != CHANNELS) {
    perror("unable to set number of channels");
    r=-1;
  }
  arg = RATE;      /* sampling rate */
  status = ioctl(fd, SOUND_PCM_READ_RATE, &arg);
  if (status == -1) {
    perror("SOUND_PCM_READ_RATE ioctl failed");return 0;
    r=-1;
  }

  return r;

#endif
}

void 
Waveaudio::openDSPforRead() 
{
#ifndef TEST_TONE
  if (fd>0) close(fd);
  
  fd = open(DEVICE, O_RDONLY);
  if (fd < 0) {
    perror("failed to open sound device");
    exit(1);
  } 
#endif
}

int 
Waveaudio::Setup()
{
  int r=0;

  r = configureDSP();
  
  // Start dsp-read/write thread
  StartThread();

  return r;
}

/* main thread */
void 
Waveaudio::Main()
{
    player_waveform_data_t data;
    
    // clear the export buffer
    memset(&data,0,sizeof(data));
    PutMsg(device_id, NULL, PLAYER_MSGTYPE_DATA,0,&data, sizeof(data),NULL);
    
    openDSPforRead();
    
    unsigned int rate = RATE;
    unsigned short depth = DEPTH;
    unsigned int bytes = BYTES;
    
    data.rate = htonl(rate);
    data.depth = htons((unsigned short)depth);
    data.samples = htonl(bytes);
    
    //int freq = 0, minfreq = 1000, maxfreq = 5000;
    
    while(1) 
      {
	pthread_testcancel();
	
	//puts( "READING" );

#ifndef TEST_TONE
	// SAMPLE DATA FROM THE DSP
	if(read(fd, &(data.data), bytes) < (ssize_t)bytes ) 
	  {
	    PLAYER_WARN("not enough data read");
	  } 
#else
	// generate a series of test tones
	double omega= freq * 2.0 * M_PI / RATE;
	double amp = 32;
	double phase = 0;
	
	freq = (int)(freq*1.1); // raise note between bursts
	
	if( freq < minfreq || freq > maxfreq ) freq = minfreq;

	usleep( 100000 );
	
	for(unsigned int i=0; i < bytes; i++) 
	  {
	    phase+=omega;
	    if (phase>M_PI*2) phase-=M_PI*2;
	    data.data[i]=(unsigned char)(127+(int)(amp * sin(phase)));
	  }	
#endif
	/*
	  printf( "rate: %d depth: %d samples: %d\n", 
	  ntohl(data.rate),
	  ntohs(data.depth),
	  ntohl(data.samples) );
	*/
	
	PutMsg(device_id,NULL, PLAYER_MSGTYPE_DATA,0,&data, sizeof(data),NULL);
      }
}

int 
Waveaudio::Shutdown()
{
  StopThread();
  if (fd>0) close(fd);
  //printf("Audio-device has been shutdown\n");
  return 0;
}

