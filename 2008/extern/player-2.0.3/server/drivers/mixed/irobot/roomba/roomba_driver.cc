/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2006 -
 *     Brian Gerkey
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

/** @ingroup drivers */
/** @{ */
/** @defgroup driver_roomba roomba
 @brief iRobot Roomba

Newer versions of the iRobot Roomba vaccum robot can be controlled by an
external computer over a serial line.  This driver supports control of
these robots.  

Note that the serial port on top of the Roomba operates at 5V, not the
RS232 standard of 12V.  This means that you cannot just plug a plain
old serial cable between the Roomba and your PC's serial port.  You need
to put a level-shifter in between them.  Or you if have a computer that
exposes serial lines at "logic level," (e.g., the Gumstix), you can use
them directly.  Check out <a href="http://www.irobot.com/hacker">iRobot's
hacker site</a> for more information, including the pinout on the Roomba's
serial port.  The <a href="http://roomba.pbwiki.com">Roomba Wiki</a>
has a howto on building an appropriate serial cable.

@par Compile-time dependencies

- none

@par Provides

The roomba driver provides the following device interfaces:

- @ref interface_position2d
  - This interface returns odometry data, and accepts velocity commands.

@par Supported configuration requests

- None

@par Configuration file options

- port (string)
  - Default: "/dev/ttyS0"
  - Serial port used to communicate with the robot.
- safe (integer)
  - Default: 1
  - Nonzero to keep the robot in "safe" mode (the robot will stop if
    the wheeldrop or cliff sensors are triggered), zero for "full" mode

@par Example

@verbatim
driver
(
  name "roomba"
  provides ["position2d:0"]
  port "/dev/ttyS2"
  safe 1
)
@endverbatim

@todo
- Add power and bumper interfaces
- Recover from a cliff/wheeldrop sensor being triggered in safe mode;
the robot goes into passive mode when this happens, which right now
requires Player to be restarted
- Add some config requests, like position geometry

@author Brian Gerkey
*/
/** @} */


#include <unistd.h>

#include <libplayercore/playercore.h>

#include "roomba_comms.h"

#define CYCLE_TIME_US 200000

class Roomba : public Driver
{
  public:
    Roomba(ConfigFile* cf, int section);

    int Setup();
    int Shutdown();

    // MessageHandler
    int ProcessMessage(MessageQueue * resp_queue, 
		       player_msghdr * hdr, 
		       void * data);

  private:
    // Main function for device thread.
    virtual void Main();

    // Serial port where the roomba is
    const char* serial_port;

    // full control or not
    bool safe;

    player_devaddr_t position_addr;
    player_devaddr_t power_addr;
    player_devaddr_t bumper_addr;

    // The underlying roomba object
    roomba_comm_t* roomba_dev;
};

// a factory creation function
Driver* Roomba_Init(ConfigFile* cf, int section)
{
  return((Driver*)(new Roomba(cf, section)));
}

// a driver registration function
void Roomba_Register(DriverTable* table)
{
  table->AddDriver("roomba", Roomba_Init);
}

Roomba::Roomba(ConfigFile* cf, int section)
        : Driver(cf,section,true,PLAYER_MSGQUEUE_DEFAULT_MAXLEN)
{
  memset(&this->position_addr,0,sizeof(player_devaddr_t));
  memset(&this->power_addr,0,sizeof(player_devaddr_t));
  memset(&this->bumper_addr,0,sizeof(player_devaddr_t));

  // Do we create a position interface?
  if(cf->ReadDeviceAddr(&(this->position_addr), section, "provides",
                        PLAYER_POSITION2D_CODE, -1, NULL) == 0)
  {
    if(this->AddInterface(this->position_addr) != 0)
    {
      this->SetError(-1);
      return;
    }
  }

  // Do we create a power interface?
  if(cf->ReadDeviceAddr(&(this->power_addr), section, "provides",
                        PLAYER_POWER_CODE, -1, NULL) == 0)
  {
    if(this->AddInterface(this->power_addr) != 0)
    {
      this->SetError(-1);
      return;
    }
  }

  // Do we create a bumper interface?
  if(cf->ReadDeviceAddr(&(this->bumper_addr), section, "provides",
                        PLAYER_BUMPER_CODE, -1, NULL) == 0)
  {
    if(this->AddInterface(this->bumper_addr) != 0)
    {
      this->SetError(-1);
      return;
    }
  }

  this->serial_port = cf->ReadString(section, "port", "/dev/ttyS0");
  this->safe = cf->ReadInt(section, "safe", 1);
  this->roomba_dev = NULL;
}

int
Roomba::Setup()
{
  this->roomba_dev = roomba_create(this->serial_port);

  if(roomba_open(this->roomba_dev, !this->safe) < 0)
  {
    roomba_destroy(this->roomba_dev);
    this->roomba_dev = NULL;
    PLAYER_ERROR("failed to connect to roomba");
    return(-1);
  }

  this->StartThread();

  return(0);
}

int
Roomba::Shutdown()
{
  this->StopThread();

  if(roomba_close(this->roomba_dev))
  {
    PLAYER_ERROR("failed to close roomba connection");
  }
  roomba_destroy(this->roomba_dev);
  this->roomba_dev = NULL;
  return(0);
}

void
Roomba::Main()
{
  for(;;)
  {
     this->ProcessMessages();

     if(roomba_get_sensors(this->roomba_dev, -1) < 0)
     {
       PLAYER_ERROR("failed to get sensor data from roomba");
       roomba_close(this->roomba_dev);
       return;
     }

     ////////////////////////////
     // Update position2d data
     player_position2d_data_t posdata;
     memset(&posdata,0,sizeof(posdata));

     posdata.pos.px = this->roomba_dev->ox;
     posdata.pos.py = this->roomba_dev->oy;
     posdata.pos.pa = this->roomba_dev->oa;

     this->Publish(this->position_addr, NULL,
                   PLAYER_MSGTYPE_DATA, PLAYER_POSITION2D_DATA_STATE,
                   (void*)&posdata, sizeof(posdata), NULL);

     ////////////////////////////
     // Update power data
     player_power_data_t powerdata;
     memset(&powerdata,0,sizeof(powerdata));

     powerdata.volts = this->roomba_dev->voltage;
     powerdata.watts = this->roomba_dev->voltage * this->roomba_dev->current;
     powerdata.joules = this->roomba_dev->charge;
     powerdata.percent = 100.0 * 
             (this->roomba_dev->charge / this->roomba_dev->capacity);
     powerdata.charging = 
             (this->roomba_dev->charging_state == ROOMBA_CHARGING_NOT) ? 0 : 1;
     powerdata.valid = (PLAYER_POWER_MASK_VOLTS |
                        PLAYER_POWER_MASK_WATTS | 
                        PLAYER_POWER_MASK_JOULES | 
                        PLAYER_POWER_MASK_PERCENT |
                        PLAYER_POWER_MASK_CHARGING);

     this->Publish(this->power_addr, NULL,
                   PLAYER_MSGTYPE_DATA, PLAYER_POWER_DATA_STATE,
                   (void*)&powerdata, sizeof(powerdata), NULL);

     ////////////////////////////
     // Update bumper data
     player_bumper_data_t bumperdata;
     memset(&bumperdata,0,sizeof(bumperdata));

     bumperdata.bumpers_count = 2;
     bumperdata.bumpers[0] = this->roomba_dev->bumper_left;
     bumperdata.bumpers[1] = this->roomba_dev->bumper_right;

     this->Publish(this->bumper_addr, NULL,
                   PLAYER_MSGTYPE_DATA, PLAYER_BUMPER_DATA_STATE,
                   (void*)&bumperdata, sizeof(bumperdata), NULL);


     usleep(CYCLE_TIME_US);
  }
}

int
Roomba::ProcessMessage(MessageQueue * resp_queue, 
		       player_msghdr * hdr, 
		       void * data)
{
  if(Message::MatchMessage(hdr,
                           PLAYER_MSGTYPE_CMD,
                           PLAYER_POSITION2D_CMD_VEL,
                           this->position_addr))
  {
    // get and send the latest motor command
    player_position2d_cmd_vel_t position_cmd;
    position_cmd = *(player_position2d_cmd_vel_t*)data;
    PLAYER_MSG2(2,"sending motor commands %f:%f", 
                position_cmd.vel.px,
                position_cmd.vel.pa);
    if(roomba_set_speeds(this->roomba_dev, 
                         position_cmd.vel.px, 
                         position_cmd.vel.pa) < 0)
    {
      PLAYER_ERROR("failed to set speeds to roomba");
    }
    return(0);
  }
  else if(Message::MatchMessage(hdr,PLAYER_MSGTYPE_REQ,
                                PLAYER_POSITION2D_REQ_GET_GEOM,
                                this->position_addr))
  {
    /* Return the robot geometry. */
    player_position2d_geom_t geom;
    // Assume that it turns about its geometric center
    geom.pose.px = 0.0;
    geom.pose.py = 0.0;
    geom.pose.pa = 0.0;

    geom.size.sl = ROOMBA_DIAMETER;
    geom.size.sw = ROOMBA_DIAMETER;

    this->Publish(this->position_addr, resp_queue,
                  PLAYER_MSGTYPE_RESP_ACK,
                  PLAYER_POSITION2D_REQ_GET_GEOM,
                  (void*)&geom, sizeof(geom), NULL);
    return(0);
  }
  else if(Message::MatchMessage(hdr,PLAYER_MSGTYPE_REQ,
                                PLAYER_BUMPER_GET_GEOM,
                                this->bumper_addr))
  {
    player_bumper_geom_t geom;

    geom.bumper_def_count = 2;

    geom.bumper_def[0].pose.px = 0.0;
    geom.bumper_def[0].pose.py = 0.0;
    geom.bumper_def[0].pose.pa = 0.0;
    geom.bumper_def[0].length = 0.0;
    geom.bumper_def[0].radius = ROOMBA_DIAMETER/2.0;

    geom.bumper_def[1].pose.px = 0.0;
    geom.bumper_def[1].pose.py = 0.0;
    geom.bumper_def[1].pose.pa = 0.0;
    geom.bumper_def[1].length = 0.0;
    geom.bumper_def[1].radius = ROOMBA_DIAMETER/2.0;

    this->Publish(this->bumper_addr, resp_queue,
                  PLAYER_MSGTYPE_RESP_ACK,
                  PLAYER_BUMPER_GET_GEOM,
                  (void*)&geom, sizeof(geom), NULL);

    return(0);
  }
  else
    return(-1);
}
