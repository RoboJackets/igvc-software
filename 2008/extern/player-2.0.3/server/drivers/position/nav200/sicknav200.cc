/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  
 *     Brian Gerkey, Kasper Stoy, Richard Vaughan, & Andrew Howard
 *                      
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
 Desc: Driver for the SICK S3000 laser
 Author: Toby Collett (based on lms200 by Andrew Howard)
 Date: 7 Nov 2000
 CVS: $Id: sicknav200.cc,v 1.1.2.1 2006/09/22 23:58:35 gerkey Exp $
*/

/** @ingroup drivers Drivers */
/** @{ */
/** @defgroup driver_sicknav200 sicknav200
 * @brief SICK NAV200 laser localisation unit

The sicknav200 driver interfaces to the NAV200 localiation unit and provides the current position
output of the device. 

Currently the driver assumes the nav200 has been correctly initialised and loaded with the 
reflector layers.

@par Compile-time dependencies

- none

@par Provides

- @ref interface_laser

@par Requires

- none

@par Configuration requests

- PLAYER_POSITION2D_REQ_GET_GEOM
  
@par Configuration file options

- port (string)
  - Default: "/dev/ttyS0"
  - Serial port to which laser is attached.  If you are using a
    USB/232 or USB/422 converter, this will be "/dev/ttyUSBx".

- pose (length tuple)
  - Default: [0.0 0.0 0.0]
  - Pose (x,y,theta) of the laser, relative to its parent object (e.g.,
    the robot to which the laser is attached).

- size (length tuple)
  - Default: [0.15 0.15]
  - Footprint (x,y) of the laser.
      
@par Example 

@verbatim
driver
(
  name "sicknav200"
  provides ["position2d:0"]
  port "/dev/ttyS0"
)
@endverbatim

@author Kathy Fung, Toby Collett, inro technologies

*/
/** @} */
  

  
#if HAVE_CONFIG_H
  #include <config.h>
#endif

#include <assert.h>
#include <math.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <arpa/inet.h> // for htons etc

#include <libplayercore/playercore.h>
#include <replace/replace.h>
extern PlayerTime* GlobalTime;

#include "nav200.h"

// The laser device class.
class SickNAV200 : public Driver
{
  public:
    
    // Constructor
    SickNAV200(ConfigFile* cf, int section);
    ~SickNAV200();

    int Setup();
    int Shutdown();

    // MessageHandler
    int ProcessMessage(MessageQueue * resp_queue, 
		       player_msghdr * hdr, 
		       void * data);
  private:

    // Main function for device thread.
    virtual void Main();

  protected:

    // Laser pose in robot cs.
    double pose[3];
    double size[2];
    
    // Name of device used to communicate with the laser
    const char *device_name;
    
    // storage for outgoing data
    player_position2d_data_t data_packet;

    // nav200 parameters
    Nav200 Laser;
    int min_radius, max_radius;
    

};

// a factory creation function
Driver* SickNAV200_Init(ConfigFile* cf, int section)
{
  return((Driver*)(new SickNAV200(cf, section)));
}

// a driver registration function
void SickNAV200_Register(DriverTable* table)
{
  table->AddDriver("sicknav200", SickNAV200_Init);
}


////////////////////////////////////////////////////////////////////////////////
// Error macros
#define RETURN_ERROR(erc, m) {PLAYER_ERROR(m); return erc;}
 
////////////////////////////////////////////////////////////////////////////////
// Constructor
SickNAV200::SickNAV200(ConfigFile* cf, int section)
    : Driver(cf, section, true, PLAYER_MSGQUEUE_DEFAULT_MAXLEN, PLAYER_POSITION2D_CODE)
{
  // Laser geometry.
  this->pose[0] = cf->ReadTupleLength(section, "pose", 0, 0.0);
  this->pose[1] = cf->ReadTupleLength(section, "pose", 1, 0.0);;
  this->pose[2] = cf->ReadTupleLength(section, "pose", 2, 0.0);;
  this->size[0] = 0.15;
  this->size[1] = 0.15;

  // Serial port
  this->device_name = strdup(cf->ReadString(section, "port", DEFAULT_PORT));

  // nav200 parameters, conver to cm
  this->min_radius = static_cast<int> (cf->ReadLength(section, "min_radius", 1) * 1000);
  this->max_radius = static_cast<int> (cf->ReadLength(section, "max_radius", 30) * 1000);

  return;
}

SickNAV200::~SickNAV200()
{
  delete device_name;
}


////////////////////////////////////////////////////////////////////////////////
// Set up the device
int SickNAV200::Setup()
{
  PLAYER_MSG1(2, "NAV200 initialising (%s)", this->device_name);
    
  // Open the terminal
  Laser.Initialise(this->device_name);
  if (!Laser.EnterStandby() || !Laser.EnterPositioning())
  {
      PLAYER_ERROR("unable to enter standby or position mode\n");
      return -1;;
  }
  if (!Laser.SetActionRadii(min_radius, max_radius))
  {
      PLAYER_ERROR("failed to set action radii\n");
      return -1;;
  }

  PLAYER_MSG0(2, "NAV200 ready");

  // Start the device thread
  StartThread();

  return 0;
}


////////////////////////////////////////////////////////////////////////////////
// Shutdown the device
int SickNAV200::Shutdown()
{
  // shutdown laser device
  StopThread();

  PLAYER_MSG0(2, "laser shutdown");
  
  return(0);
}


int 
SickNAV200::ProcessMessage(MessageQueue * resp_queue, 
                           player_msghdr * hdr,
                           void * data)
{
  if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
                                 PLAYER_POSITION2D_REQ_GET_GEOM,
                                 this->device_addr))
  {
    if(hdr->size != 0)
    {
      PLAYER_ERROR2("request is wrong length (%d != %d); ignoring",
                    hdr->size, 0);
      return(PLAYER_MSGTYPE_RESP_NACK);
    }
    player_position2d_geom_t geom;
    geom.pose.px = this->pose[0];
    geom.pose.py = this->pose[1];
    geom.pose.pa = this->pose[2];
    geom.size.sl = this->size[0];
    geom.size.sw = this->size[1];

    this->Publish(this->device_addr,
                  resp_queue,
                  PLAYER_MSGTYPE_RESP_ACK,
                  PLAYER_POSITION2D_REQ_GET_GEOM,
                  (void*)&geom, sizeof(geom), NULL);
    return(0);
  }

  // Don't know how to handle this message.
  return(-1);
}

////////////////////////////////////////////////////////////////////////////////
// Main function for device thread
void SickNAV200::Main() 
{
  LaserPos Reading;
  for(;;)
  {
    // test if we are supposed to cancel
    pthread_testcancel();
    
    // process any pending messages
    ProcessMessages();
    
	// get update and publish result
    if(Laser.GetPositionAuto(Reading))
    {
      data_packet.pos.px = static_cast<double> (Reading.pos.x)/1000;
      data_packet.pos.py = static_cast<double> (Reading.pos.y)/1000;
      data_packet.pos.pa = static_cast<double> (Reading.orientation)/1000;

      this->Publish(this->device_addr,
                   NULL,
                   PLAYER_MSGTYPE_DATA,
                   PLAYER_POSITION2D_DATA_STATE,
                   (void*)&data_packet, sizeof(data_packet), NULL);
    }
    else
    {
      PLAYER_WARN("Failed to get reading from laser scanner\n");
      usleep(100000);
    }
  }
}



