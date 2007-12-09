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
/** @defgroup driver_mixer mixer
@brief Controls mixer levels (volume, balance, etc) on sound hardware.

@todo This driver is currently disabled because it needs to be updated to
the Player 2.0 API.

@todo Write a description

@par Compile-time dependencies

- &lt;sys/soundcard.h&gt;

@par Provides

- @ref interface_audiomixer

@par Requires

- none

@par Configuration requests

TODO

@par Configuration file options

- device (string)
  - Default: "/dev/mixer"
  - Mixer device to use.

@par Example

@verbatim
driver
(
  name "mixer"
  provides ["audiomixer:0"]
  device "/dev/mixer""
)
@endverbatim

@author Nate Koenig

*/

/** @} */


#include <sys/soundcard.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <netinet/in.h>

#include "playercommon.h"
#include "drivertable.h"
#include "driver.h"
#include "error.h"
#include "player.h"

#define DEFAULT_DEVICE "/dev/mixer"

class Mixer : public Driver
{
  public:
    // Constructor
    Mixer( ConfigFile* cf, int section);

    int Setup();
    int Shutdown();

  private:

    // Open or close the device
    int OpenDevice( int flag );
    int CloseDevice();

    // The main loop
    virtual void Main();

    // Process incoming messages from clients 
    int ProcessMessage(ClientData * client, player_msghdr * hdr, uint8_t * data, uint8_t * resp_data, int * resp_len);


    int Write( int dev, int num );
    int Read( int dev, int& num );

    int mixerFD; // File descriptor for the device
    const char* deviceName; // Name of the device( ex: "/dev/mixer" )
};

Mixer::Mixer( ConfigFile* cf, int section)
        : Driver(cf, section, true, PLAYER_MSGQUEUE_DEFAULT_MAXLEN, PLAYER_AUDIOMIXER_CODE, PLAYER_ALL_MODE)
{
  deviceName = cf->ReadString(section,"device",DEFAULT_DEVICE);
}

Driver* Mixer_Init( ConfigFile* cf, int section)
{
  return((Driver*)(new Mixer(cf, section)));
}

void Mixer_Register(DriverTable* table)
{
  table->AddDriver("mixer", Mixer_Init );
}

int Mixer::Setup()
{
  mixerFD = open(deviceName,O_RDWR);

  StartThread();

  puts("mixer ready");
  return 0;
}

int Mixer::Shutdown()
{
  StopThread();
  close(mixerFD);

  puts("Mixer has been shutdown");

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Process an incoming message
int Mixer::ProcessMessage(ClientData * client, player_msghdr * hdr, uint8_t * data, uint8_t * resp_data, int * resp_len)
{
  assert(hdr);
  assert(data);
  assert(resp_data);
  assert(resp_len);
  assert(*resp_len == PLAYER_MAX_MESSAGE_SIZE);
  
  *resp_len = 0;

  int vol;

  if (MatchMessage(hdr, PLAYER_MSGTYPE_REQ, PLAYER_AUDIOMIXER_GET_LEVELS, device_id))
  {
    assert(hdr->size == 0);
    Lock();
    player_audiomixer_config_t * config = reinterpret_cast<player_audiomixer_config_t *> (resp_data);

    this->Read(SOUND_MIXER_VOLUME,vol);
    config->masterLeft = htons( vol & 0xFF );
    config->masterRight = htons( (vol>>8) & 0xFF );

    this->Read(SOUND_MIXER_PCM,vol);
    config->pcmLeft = htons( vol & 0xFF );
    config->pcmRight = htons( (vol>>8) & 0xFF );

    this->Read(SOUND_MIXER_LINE,vol);
    config->lineLeft = htons( vol & 0xFF );
    config->lineRight = htons( (vol>>8) & 0xFF );

    this->Read(SOUND_MIXER_MIC,vol);
    config->micLeft = htons( vol & 0xFF );
    config->micRight = htons( (vol>>8) & 0xFF );

    this->Read(SOUND_MIXER_IGAIN,vol);
    config->iGain = htons( vol );

    this->Read(SOUND_MIXER_OGAIN,vol);
    config->oGain= htons( vol );

    Unlock();
    *resp_len = sizeof(player_audiomixer_config_t);
    return PLAYER_MSGTYPE_RESP_ACK;
  }

  if (MatchMessage(hdr, PLAYER_MSGTYPE_CMD, PLAYER_AUDIOMIXER_SET_MASTER, device_id))
  {
    assert(hdr->size == sizeof(player_audiomixer_cmd_t));
    player_audiomixer_cmd_t * cmd = reinterpret_cast<player_audiomixer_cmd_t *> (data);
    vol = ( ntohs(cmd->left)&0xFF) | ( ntohs(cmd->right) << 8);
    this->Write( SOUND_MIXER_VOLUME, vol );
    return 0;
  }
  
  if (MatchMessage(hdr, PLAYER_MSGTYPE_CMD, PLAYER_AUDIOMIXER_SET_PCM, device_id))
  {
    assert(hdr->size == sizeof(player_audiomixer_cmd_t));
    player_audiomixer_cmd_t * cmd = reinterpret_cast<player_audiomixer_cmd_t *> (data);
    vol = ( ntohs(cmd->left)&0xFF) | ( ntohs(cmd->right) << 8);
    this->Write( SOUND_MIXER_PCM, vol );
    return 0;
  }
  
  if (MatchMessage(hdr, PLAYER_MSGTYPE_CMD, PLAYER_AUDIOMIXER_SET_LINE, device_id))
  {
    assert(hdr->size == sizeof(player_audiomixer_cmd_t));
    player_audiomixer_cmd_t * cmd = reinterpret_cast<player_audiomixer_cmd_t *> (data);
    vol = ( ntohs(cmd->left)&0xFF) | ( ntohs(cmd->right) << 8);
    this->Write( SOUND_MIXER_LINE, vol );
    return 0;
  }
  
  if (MatchMessage(hdr, PLAYER_MSGTYPE_CMD, PLAYER_AUDIOMIXER_SET_MIC, device_id))
  {
    assert(hdr->size == sizeof(player_audiomixer_cmd_t));
    player_audiomixer_cmd_t * cmd = reinterpret_cast<player_audiomixer_cmd_t *> (data);
    vol = ( ntohs(cmd->left)&0xFF) | ( ntohs(cmd->right) << 8);
    this->Write( SOUND_MIXER_MIC, vol );
    return 0;
  }
  
  if (MatchMessage(hdr, PLAYER_MSGTYPE_CMD, PLAYER_AUDIOMIXER_SET_IGAIN, device_id))
  {
    assert(hdr->size == sizeof(player_audiomixer_cmd_t));
    player_audiomixer_cmd_t * cmd = reinterpret_cast<player_audiomixer_cmd_t *> (data);
    this->Write( SOUND_MIXER_IGAIN, ntohs(cmd->left) );
    return 0;
  }
 
  if (MatchMessage(hdr, PLAYER_MSGTYPE_CMD, PLAYER_AUDIOMIXER_SET_OGAIN, device_id))
  {
    assert(hdr->size == sizeof(player_audiomixer_cmd_t));
    player_audiomixer_cmd_t * cmd = reinterpret_cast<player_audiomixer_cmd_t *> (data);
    this->Write( SOUND_MIXER_OGAIN, ntohs(cmd->left) );
    return 0;
  }

  return -1;
}


void Mixer::Main()
{
/*  void* client;
  unsigned char configBuffer[PLAYER_MAX_REQREP_SIZE];
  unsigned char cmdBuffer[sizeof(player_audiodsp_cmd)];
  int len;
  int vol;

  player_audiomixer_cmd_t mixerCmd;*/

  while(true)
  {
    // test if we are suppose to cancel
    pthread_testcancel();
    
    // process any pending messages
    ProcessMessages();
    
    // make sure the thread is idle sometimes
    usleep(10);
  }
}

int Mixer::Write(int dev, int num)
{
  int result;

  if( (result = ioctl(mixerFD,MIXER_WRITE(dev), &num)) == -1 )
    PLAYER_ERROR("write failed");

  return result;
}

int Mixer::Read(int dev, int& num)
{
  int result;

  if( (result = ioctl(mixerFD,MIXER_READ(dev), &num)) == -1 )
    PLAYER_ERROR("read failed");

  return result;
}
