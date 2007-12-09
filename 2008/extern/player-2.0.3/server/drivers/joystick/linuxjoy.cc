/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2003  
 *     Brian Gerkey, Andrew Howard
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
 * Desc: Read data from a standard linux joystick
 * Author: Andrew Howard
 * Date: 25 July 2004
 * CVS: $Id: linuxjoy.cc,v 1.30 2006/02/27 18:19:03 gerkey Exp $
 *
 */

/** @ingroup drivers */
/** @{ */
/** @defgroup driver_linuxjoystick linuxjoystick
 * @brief Linux joystick

The linuxjoystick driver reads data from a standard Linux joystick and
provides the data via the @ref interface_joystick interface.
This driver can also control a @ref interface_position2d device by
converting joystick positions to velocity commands.

@par Compile-time dependencies

- &lt;linux/joystick.h&gt;

@par Provides

- @ref interface_joystick : joystick data
- @ref interface_position2d : joystick data represented as 2-D 
  position data.  Raw X- and Y-axis values are reported as xpos and ypos in the 
  position packet (all other fields are zero).

@par Requires

- @ref interface_position2d : if present, joystick positions will be
  interpreted as velocities and sent as commands to this position2d device.
  See also max_xspeed, max_yawspeed, and deadman_button options below.

@par Configuration requests

- None

@par Configuration file options

- port (string)
  - Default: "/dev/js0"
  - The joystick to be used.
- axes (integer tuple)
  - Default: [0 1]
  - Which joystick axes to call the "X" and "Y" axes, respectively.
- axis_maxima (integer tuple)
  - Default: [32767 32767]
  - Maximum absolute values attainable on the X and Y axes, respectively.
- axis_minima (integer tuple)
  - Default: [0 0]
  - Minimum values on the X and Y axes, respectively.  Anything smaller
    in absolute value than this limit will be reported as zero.
    Useful for implementing a dead zone on a touchy joystick.
- deadman_button (integer)
  - Default: -1
  - When controlling a @ref interface_position2d device, if deadman_button is 
    >= 0, this joystick button must be depressed for commands to be 
    sent to that device.
- max_xspeed (length / sec)
  - Default: 0.5 m/sec
  - The maximum absolute translational velocity to be used when commanding a
    position device.
- max_yawspeed (angle / sec)
  - Default: 30 deg/sec
  - The maximum absolute rotational velocity to be used when commanding a
    position device.
- timeout (float)
  - Default: 5.0
  - Time (in seconds) since receiving a new joystick event after which
    the underlying position device will be stopped, for safety.  Set to
    0.0 for no timeout.

@par Examples

Basic configuration

@verbatim
driver
(
  name "linuxjoystick"
  provides ["joystick:0"]
  port "/dev/js0"
)
@endverbatim

Provide a position interface, instead of a joystick interface.

@verbatim
driver
(
  name "linuxjoystick"
  provides ["position:0"]
  port "/dev/js0"
)
@endverbatim

Controlling a Pioneer, plus remapping joystick axes and setting various
limits.

@verbatim
driver
(
  name "p2os"
  provides ["odometry::position:0"]
  port "/dev/usb/tts/0"
)

driver
(
  name "linuxjoystick"
  provides ["joystick:0"]
  requires ["odometry::position:0"]
  max_yawspeed 50
  max_xspeed 0.5
  axes [3 4]
  axis_minima [5000 5000]
  port "/dev/js0"
  alwayson 1
)
@endverbatim

@todo
Add support for continuously sending commands, which might be needed for 
position devices that use watchdog timers.

@author Andrew Howard, Brian Gerkey, Paul Osmialowski

*/
/** @} */

#include <netinet/in.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <sys/time.h>
#include <fcntl.h>
#include <math.h>
#include <linux/joystick.h>

#include <replace/replace.h> // for poll(2)
#include <libplayercore/playercore.h>
#include <libplayercore/error.h>

extern PlayerTime *GlobalTime;

#define XAXIS 0
#define YAXIS 1

#define MAX_XSPEED 0.5
#define MAX_YAWSPEED DTOR(30.0)
#define AXIS_MAX ((int16_t) 32767)

////////////////////////////////////////////////////////////////////////////////
// The class for the driver
class LinuxJoystick : public Driver
{
  // Constructor; need that
  public: LinuxJoystick(ConfigFile* cf, int section);

  // Must implement the following methods.
  public: int Setup();
  public: int Shutdown();

  // Main function for device thread.
  private: virtual void Main();

  public: virtual int ProcessMessage(MessageQueue* resp_queue, 
                                     player_msghdr * hdr, 
                                     void * data) {return -1;}

  // Read the joystick
  private: void ReadJoy();

  // Write new data to server
  private: void RefreshData();

  // Check for new configuration requests
  //private: void CheckConfig();

  // Put new position command
  private: void PutPositionCommand();

  // Joystick device
  private: player_devaddr_t joystick_addr;
  private: const char *dev;
  private: int fd;
  private: int16_t xpos, ypos;
  private: uint16_t buttons;
  private: int xaxis_max, yaxis_max;
  private: int xaxis_min, yaxis_min;
  private: double timeout;
  private: struct timeval lastread;

  // Position device
  private: player_devaddr_t position_addr;
  private: player_position2d_data_t pos_data;

  // These are used when we send commands to a position device
  private: double max_xspeed, max_yawspeed;
  private: int xaxis, yaxis;
  private: int deadman_button;
  private: player_devaddr_t cmd_position_addr;
  private: Device* position;

  // Joystick
  private: player_joystick_data_t joy_data;
};


////////////////////////////////////////////////////////////////////////////////
// A factory creation function, declared outside of the class so that it
// can be invoked without any object context (alternatively, you can
// declare it static in the class).  In this function, we create and return
// (as a generic Driver*) a pointer to a new instance of this driver.
Driver* LinuxJoystick_Init(ConfigFile* cf, int section)
{
  // Create and return a new instance of this driver
  return ((Driver*) (new LinuxJoystick(cf, section)));
}


////////////////////////////////////////////////////////////////////////////////
// A driver registration function, again declared outside of the class so
// that it can be invoked without object context.  In this function, we add
// the driver into the given driver table, indicating which interface the
// driver can support and how to create a driver instance.
void LinuxJoystick_Register(DriverTable* table)
{
  table->AddDriver("linuxjoystick", LinuxJoystick_Init);
}


////////////////////////////////////////////////////////////////////////////////
// Constructor.  Retrieve options from the configuration file and do any
// pre-Setup() setup.
LinuxJoystick::LinuxJoystick(ConfigFile* cf, int section) : Driver(cf, section)
{
  // zero ids, so that we'll know later which interfaces were requested
  memset(&this->cmd_position_addr, 0, sizeof(player_devaddr_t));
  memset(&this->position_addr, 0, sizeof(player_devaddr_t));
  memset(&this->joystick_addr, 0, sizeof(player_devaddr_t));

  // Do we create a position interface?
  if(cf->ReadDeviceAddr(&(this->position_addr), section, "provides",
                        PLAYER_POSITION2D_CODE, -1, NULL) == 0)
  {
    if(this->AddInterface(this->position_addr))
    {
      this->SetError(-1);    
      return;
    }
  }
  // Do we create a joystick interface?
  if(cf->ReadDeviceAddr(&(this->joystick_addr), section, "provides",
                        PLAYER_JOYSTICK_CODE, -1, NULL) == 0)
  {
    if(this->AddInterface(this->joystick_addr))
    {
      this->SetError(-1);    
      return;
    }
  }

  this->dev = cf->ReadString(section, "port", "/dev/js0");
  this->xaxis = cf->ReadTupleInt(section,"axes", 0, XAXIS);
  this->yaxis = cf->ReadTupleInt(section,"axes", 1, YAXIS);
  this->deadman_button = cf->ReadInt(section,"deadman", -1);
  this->xaxis_max = cf->ReadTupleInt(section, "axis_maxima", 0, AXIS_MAX);
  this->yaxis_max = cf->ReadTupleInt(section, "axis_maxima", 1, AXIS_MAX);
  this->xaxis_min = cf->ReadTupleInt(section, "axis_minima", 0, 0);
  this->yaxis_min = cf->ReadTupleInt(section, "axis_minima", 1, 0);

  // Do we talk to a position device?
  if(cf->GetTupleCount(section, "requires"))
  {
    if(cf->ReadDeviceAddr(&(this->cmd_position_addr), section, "requires", 
                          PLAYER_POSITION2D_CODE, -1, NULL) == 0)
    {
      this->max_xspeed = cf->ReadLength(section, "max_xspeed", MAX_XSPEED);
      this->max_yawspeed = cf->ReadAngle(section, "max_yawspeed", MAX_YAWSPEED);
      this->timeout = cf->ReadFloat(section, "timeout", 5.0);
    }
  }

  return;
}

////////////////////////////////////////////////////////////////////////////////
// Set up the device.  Return 0 if things go well, and -1 otherwise.
int LinuxJoystick::Setup()
{
  // Open the joystick device
  this->fd = open(this->dev, O_RDONLY);
  if (this->fd < 1)
  {
    PLAYER_ERROR2("unable to open joystick [%s]; %s",
                  this->dev, strerror(errno));
    return -1;
  }

  this->lastread.tv_sec = this->lastread.tv_usec = 0;

  // If we're asked, open the position device
  if(this->cmd_position_addr.interf)
  {
    if(!(this->position = deviceTable->GetDevice(this->cmd_position_addr)))
    {
      PLAYER_ERROR("unable to locate suitable position device");
      return(-1);
    }
    if(this->position->Subscribe(this->InQueue) != 0)
    {
      PLAYER_ERROR("unable to subscribe to position device");
      return(-1);
    }

    // Enable the motors
    player_position2d_power_config_t motorconfig;
    motorconfig.state = 1;
    Message* msg;
    if(!(msg = this->position->Request(this->InQueue,
                                       PLAYER_MSGTYPE_REQ, 
                                       PLAYER_POSITION2D_REQ_MOTOR_POWER,
                                       (void*)&motorconfig,
                                       sizeof(motorconfig),NULL,false)))
    {
      PLAYER_WARN("failed to enable motors");
    }
    else
      delete msg;

    // Stop the robot
    player_position2d_cmd_vel_t cmd;
    memset(&cmd,0,sizeof(cmd));
    this->position->PutMsg(this->InQueue,
                           PLAYER_MSGTYPE_CMD,
                           PLAYER_POSITION2D_CMD_VEL,
                           (void*)&cmd, sizeof(player_position2d_cmd_vel_t),
                           NULL);
  }
  
  this->xpos = this->ypos = 0;
  
  // Start the device thread; spawns a new thread and executes
  // LinuxJoystick::Main(), which contains the main loop for the driver.
  this->StartThread();

  return(0);
}


////////////////////////////////////////////////////////////////////////////////
// Shutdown the device
int LinuxJoystick::Shutdown()
{
  // Stop and join the driver thread
  this->StopThread();

  if(this->cmd_position_addr.interf)
    this->position->Unsubscribe(this->InQueue);

  // Close the joystick
  close(this->fd);

  return(0);
}


////////////////////////////////////////////////////////////////////////////////
// Main function for device thread
void LinuxJoystick::Main() 
{
  // The main loop; interact with the device here
  while (true)
  {
    // test if we are supposed to cancel
    pthread_testcancel();

    // Run and process output
    this->ReadJoy();
    
    // Write outgoing data
    this->RefreshData();

    // Send new commands to position device
    if(this->cmd_position_addr.interf)
    {
      if((this->deadman_button < 0) ||
         ((this->buttons >> this->deadman_button) & 0x01))
      {
        this->PutPositionCommand();
      }
#if 0
      else
      {
        player_position2d_cmd_vel_t cmd;
        memset(&cmd,0,sizeof(cmd));
        this->position->PutMsg(this->InQueue,
                               PLAYER_MSGTYPE_CMD,
                               PLAYER_POSITION2D_CMD_VEL,
                               (void*)&cmd, sizeof(player_position2d_cmd_vel_t),
                               NULL);
      }
#endif
    }
  }
  return;
}


////////////////////////////////////////////////////////////////////////////////
// Read the joystick
void LinuxJoystick::ReadJoy()
{
  struct pollfd fd;
  struct js_event event;
  int count;
  
  fd.fd = this->fd;
  fd.events = POLLIN | POLLHUP;
  fd.revents = 0;

  count = poll(&fd, 1, 10);
  if (count < 0)
    PLAYER_ERROR1("poll returned error [%s]", strerror(errno));
  else if(count > 0)
  {
    // get the next event from the joystick
    read(this->fd, &event, sizeof(struct js_event));

    //printf( "value % d type %u  number %u state %X \n", 
    //        event.value, event.type, event.number, this->joy_data.buttons );

    // Update buttons
    if ((event.type & ~JS_EVENT_INIT) == JS_EVENT_BUTTON)
    {
      if (event.value)
        this->buttons |= (1 << event.number);
      else
        this->buttons &= ~(1 << event.number);
    }

    // ignore the startup events
    if (event.type & JS_EVENT_INIT)
      return;

    switch( event.type )
    {
      case JS_EVENT_AXIS:
        {
          if(event.number == this->xaxis)
          {
            this->xpos = event.value;
            if(abs(this->xpos) < this->xaxis_min)
              this->xpos = 0;
            GlobalTime->GetTime(&this->lastread);
          }
          else if(event.number == this->yaxis)
          {
            this->ypos = event.value;
            if(abs(this->ypos) < this->yaxis_min)
              this->ypos = 0;
            GlobalTime->GetTime(&this->lastread);
          }
        }	  
        break;
    }
  }
      
  return;
}


////////////////////////////////////////////////////////////////////////////////
// Send new data out
void LinuxJoystick::RefreshData()
{
  if(this->joystick_addr.interf)
  {
    memset(&(this->joy_data),0,sizeof(player_joystick_data_t));
    this->joy_data.xpos = this->xpos;
    this->joy_data.ypos = this->ypos;
    this->joy_data.xscale = this->xaxis_max;
    this->joy_data.yscale = this->yaxis_max;
    this->joy_data.buttons = this->buttons;
    this->Publish(this->joystick_addr, NULL,
                  PLAYER_MSGTYPE_DATA, PLAYER_JOYSTICK_DATA_STATE,
                  (void*)&this->joy_data, sizeof(this->joy_data), NULL);
  }

  if(this->position_addr.interf)
  {
    memset(&(this->pos_data),0,sizeof(player_position2d_data_t));
    this->pos_data.pos.px = this->xpos;
    this->pos_data.pos.py = -this->ypos;
    this->Publish(this->position_addr, NULL,
                  PLAYER_MSGTYPE_DATA, PLAYER_POSITION2D_DATA_STATE,
                  (void*)&this->pos_data, sizeof(this->pos_data), NULL);
  }
}

// command the robot
void LinuxJoystick::PutPositionCommand()
{
  double scaled_x, scaled_y;
  double xspeed, yawspeed;
  player_position2d_cmd_vel_t cmd;
  struct timeval curr;
  double diff;

  scaled_x = this->xpos / (double) this->xaxis_max;
  scaled_y = this->ypos / (double) this->yaxis_max;

  // sanity check
  if((scaled_x > 1.0) || (scaled_x < -1.0))
  {
    PLAYER_ERROR2("X position (%d) outside of axis max (+-%d); ignoring", 
                  this->xpos, this->xaxis_max);
    return;
  }
  if((scaled_y > 1.0) || (scaled_y < -1.0))
  {
    PLAYER_ERROR2("Y position (%d) outside of axis max (+-%d); ignoring", 
                  this->ypos, this->yaxis_max);
    return;
  }

  // Note that joysticks use X for side-to-side and Y for forward-back, and
  // also that their axes are backwards with respect to intuitive driving
  // controls.
  xspeed = -scaled_y * this->max_xspeed;
  yawspeed = -scaled_x * this->max_yawspeed;

  // Make sure we've gotten a joystick fairly recently.
  GlobalTime->GetTime(&curr);
  diff = (curr.tv_sec - curr.tv_usec/1e6) -
          (this->lastread.tv_sec - this->lastread.tv_usec/1e6);
  if(this->timeout && (diff > this->timeout) && (xspeed || yawspeed))
  {
    PLAYER_WARN("Timeout on joystick; stopping robot");
    xspeed = yawspeed = 0.0;
  }

  PLAYER_MSG2(2,"sending speeds: (%f,%f)", xspeed, yawspeed);

  memset(&cmd,0,sizeof(cmd));
  cmd.vel.px = xspeed;
  cmd.vel.pa = yawspeed;
  //cmd.type=0;
  cmd.state=1;
  this->position->PutMsg(this->InQueue,
                         PLAYER_MSGTYPE_CMD,
                         PLAYER_POSITION2D_CMD_VEL,
                         (void*)&cmd, sizeof(player_position2d_cmd_vel_t),
                         NULL);
}

