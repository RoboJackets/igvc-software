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
 * Desc: Driver for writing log files.
 * Author: Andrew Howard
 * Date: 14 Jun 2003
 * CVS: $Id: writelog.cc,v 1.71.2.1 2006/09/22 23:58:35 gerkey Exp $
 *
 */

/** @ingroup drivers */
/** @{ */
/** @defgroup driver_writelog writelog
 * @brief Logging data

The writelog driver will write data from another device to a log file.
Each data message is written to a separate line.  The format for the 
file is given in the 
@ref tutorial_datalog "data logging tutorial".

The @ref driver_readlog driver can be used to replay the data
(to client programs, the replayed data will appear to come from the
real sensors).

The writelog driver logs data independently of any client connections to
the devices that it is logging.  As long as it's enabled and recording,
the writelog driver records data from the specified list of devices
at the rate that new data is produced by the first device in that list
(so put the fastest one first).  Commands are not logged.

For help in remote-controlling logging, try @ref util_playervcr.
Note that you must declare a @ref interface_log device to allow
logging control.

Note that unless you plan to remote-control this driver via the @ref
interface_log interface (e.g., using @ref util_playervcr),
you should specify the @p alwayson option in the configuration file so
that logging start when Player starts.

@par Compile-time dependencies

- none

@par Provides

- @ref interface_log : can be used to turn logging on/off

@par Requires

The writelog driver takes as input a list of devices to log data from.
The driver with the <b>highest data rate</b> should be placed first in the list.
The writelog driver can will log data from the following interfaces:

- @ref interface_laser
- @ref interface_sonar
- @ref interface_position2d
- @ref interface_ptz
- @ref interface_wifi
- @ref interface_wsn

The following interfaces are supported in principle but are currently
disabled because they need to be updated:

- @ref interface_blobfinder
- @ref interface_camera
- @ref interface_fiducial
- @ref interface_gps
- @ref interface_joystick
- @ref interface_position3d
- @ref interface_power
- @ref interface_truth

@par Configuration requests

- PLAYER_LOG_SET_WRITE_STATE
- PLAYER_LOG_GET_STATE
- PLAYER_LOG_SET_FILENAME

@par Configuration file options

- filename (string)
  - Default: "writelog_YYYY_MM_DD_HH_MM.log", where YYYY is the year,
    MM is the month, etc.
  - Name of logfile.
- autorecord (integer)
  - Default: 0
  - Default log state; set to 1 for continous logging.
- camera_save_images (integer)
  - Default: 0
  - Save camera data to image files as well as to the log file.  The image
    files are named "writelog_YYYY_MM_DD_HH_MM_camera_II_NNNNNNN.pnm",
    where II is the device index and NNNNNNN is the frame number.

@par Example

@verbatim
# Log data from laser:0 position2d:0 to "mydata.log"
driver
(
  name "writelog"
  filename "mydata.log"
  requires ["laser:0" "position2d:0"]
  provides ["log:0"]
  alwayson 1
  autorecord 1
)
@endverbatim

@author Andrew Howard

*/
/** @} */

#ifdef HAVE_CONFIG_H
  #include "config.h"
#endif

#include <assert.h>
#include <ctype.h>
#include <errno.h>
#include <sys/types.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <libplayercore/playercore.h>

#include "encode.h"

// Utility class for storing per-device info
struct WriteLogDevice
{
  public: player_devaddr_t addr;
  public: Device *device;
  public: int cameraFrame;
};


// The logfile driver
class WriteLog: public Driver
{
  /// Constructor
  public: WriteLog(ConfigFile* cf, int section);

  /// Destructor
  public: ~WriteLog();

  // MessageHandler
  public: virtual int ProcessMessage(MessageQueue * resp_queue,
                                     player_msghdr * hdr,
                                     void * data);

  /// Initialize the driver
  public: virtual int Setup();

  /// Finalize the driver
  public: virtual int Shutdown();

  // Device thread
  private: virtual void Main(void);

  // Open this->filename, save the resulting FILE* to this->file, and
  // write in the logfile header.
  private: int OpenFile();

  // Request and write geometries
  private: void WriteGeometries();

  // Flush and close this->file
  private: void CloseFile();

  // Write data to file
  private: void Write(WriteLogDevice *device,
                      player_msghdr_t* hdr, void *data);

  // Write laser data to file
  private: int WriteLaser(player_msghdr_t* hdr, void *data);

  // Write position data to file
  private: int WritePosition(player_msghdr_t* hdr, void *data);

  // Write PTZ data to file
  private: int WritePTZ(player_msghdr_t* hdr, void *data);

  // Write sonar data to file
  private: int WriteSonar(player_msghdr_t* hdr, void *data);

  // Write wifi data to file
  private: int WriteWiFi(player_msghdr_t* hdr, void *data);

  // Write WSN data to file
  private: int WriteWSN(player_msghdr_t* hdr, void *data);
#if 0
  // Write blobfinder data to file
  private: void WriteBlobfinder(player_blobfinder_data_t *data);

  // Write camera data to file
  private: void WriteCamera(player_camera_data_t *data);

  // Write camera data to image file as well
  private: void WriteCameraImage(WriteLogDevice *device, player_camera_data_t *data, struct timeval *ts);

  // Write fiducial data to file
  private: void WriteFiducial(player_fiducial_data_t *data);

  // Write GPS data to file
  private: void WriteGps(player_gps_data_t *data);

  // Write joystick data to file
  private: void WriteJoystick(player_joystick_data_t *data);

  // Write position3d data to file
  private: void WritePosition3d(player_position3d_data_t *data);

  // Write power data to file
  private: void WritePower(player_power_data_t *data);

#endif

  // File to write data to
  private: char default_basename[1024];
  private: char default_filename[1024];
  private: char filename[1024];
  private: FILE *file;

  // Subscribed device list
  private: int device_count;
  private: WriteLogDevice devices[1024];

  // Is writing enabled? (client can start/stop)
  private: bool enable;
  private: bool enable_default;

  // Save camera frames to image files as well?
  private: bool cameraSaveImages;
};


////////////////////////////////////////////////////////////////////////////
// Pointer to the one-and-only time
extern PlayerTime* GlobalTime;
extern int global_playerport;


////////////////////////////////////////////////////////////////////////////
// Create a driver for reading log files
Driver* WriteLog_Init(ConfigFile* cf, int section)
{
  return ((Driver*) (new WriteLog(cf, section)));
}


////////////////////////////////////////////////////////////////////////////
// Device factory registration
void WriteLog_Register(DriverTable* table)
{
  table->AddDriver("writelog", WriteLog_Init);
  return;
}


////////////////////////////////////////////////////////////////////////////
// Constructor
WriteLog::WriteLog(ConfigFile* cf, int section)
    : Driver(cf, section, true, PLAYER_MSGQUEUE_DEFAULT_MAXLEN, PLAYER_LOG_CODE)
{
  int i;
  player_devaddr_t addr;
  WriteLogDevice *device;
  time_t t;
  struct tm *ts;

  this->file = NULL;

  // Construct default filename from date and time.  Note that we use
  // the system time, *not* the Player time.  I think that this is the
  // correct semantics for working with simulators.
  time(&t);
  ts = localtime(&t);
  strftime(this->default_basename, sizeof(this->default_filename),
           "writelog_%Y_%m_%d_%H_%M", ts);
  snprintf(this->default_filename, sizeof(this->default_filename), "%s.log",
           this->default_basename);

  // Let user override default filename
  strcpy(this->filename,
         cf->ReadString(section, "filename", this->default_filename));

  // Default enabled?
  if(cf->ReadInt(section, "autorecord", 1) > 0)
    this->enable_default = true;
  else
    this->enable_default = false;

  this->device_count = 0;

  // Get a list of input devices
  for (i = 0; i < cf->GetTupleCount(section, "requires"); i++)
  {
    if (cf->ReadDeviceAddr(&addr, section, "requires", -1, i, NULL) != 0)
    {
      this->SetError(-1);
      return;
    }

    // Add to our device table
    assert(this->device_count < (int) (sizeof(this->devices) / sizeof(this->devices[0])));
    device = this->devices + this->device_count++;
    device->addr = addr;
    device->device = NULL;
    device->cameraFrame = 0;
  }

  // Camera specific settings
  this->cameraSaveImages = cf->ReadInt(section, "camera_save_images", 0);

  return;
}


////////////////////////////////////////////////////////////////////////////
// Destructor
WriteLog::~WriteLog()
{
  return;
}


////////////////////////////////////////////////////////////////////////////
// Initialize driver
int WriteLog::Setup()
{
  int i;
  WriteLogDevice *device;

  // Subscribe to the underlying devices
  for (i = 0; i < this->device_count; i++)
  {
    device = this->devices + i;

    device->device = deviceTable->GetDevice(device->addr);
    if (!device->device)
    {
      PLAYER_ERROR3("unable to locate device [%d:%s:%d] for logging",
                    device->addr.robot,
                    ::lookup_interface_name(0, device->addr.interf),
                    device->addr.index);
      return -1;
    }
    if(device->device->Subscribe(this->InQueue) != 0)
    {
      PLAYER_ERROR("unable to subscribe to device for logging");
      return -1;
    }
  }

  // Enable/disable logging, according to default set in config file
  this->enable = this->enable_default;

  // Start device thread
  this->StartThread();

  return 0;
}


////////////////////////////////////////////////////////////////////////////
// Finalize the driver
int WriteLog::Shutdown()
{
  int i;
  WriteLogDevice *device;

  // Stop the device thread
  this->StopThread();

  // Close the file
  this->CloseFile();

  // Unsubscribe to the underlying devices
  for (i = 0; i < this->device_count; i++)
  {
    device = this->devices + i;

    // Unsbscribe from the underlying device
    device->device->Unsubscribe(this->InQueue);
    device->device = NULL;
  }

  return 0;
}

int
WriteLog::OpenFile()
{
  // Open the file
  this->file = fopen(this->filename, "w+");
  if(this->file == NULL)
  {
    PLAYER_ERROR2("unable to open [%s]: %s\n", this->filename, strerror(errno));
    return(-1);
  }

  // Write the file header
  fprintf(this->file, "## Player version %s \n", VERSION);
  fprintf(this->file, "## File version %s \n", "0.3.0");

  fprintf(this->file, "## Format: \n");
  fprintf(this->file, "## - Messages are newline-separated\n");
  fprintf(this->file, "## - Common header to each message is:\n");
  fprintf(this->file, "##   time     host   robot  interface index  type   subtype\n");
  fprintf(this->file, "##   (double) (uint) (uint) (string)  (uint) (uint) (uint)\n");
  fprintf(this->file, "## - Following the common header is the message payload \n");

  this->WriteGeometries();

  return(0);
}

////////////////////////////////////////////////////////////////////////////
// Request and write geometries
void WriteLog::WriteGeometries()
{
  // Subscribe to the underlying devices
  for (int i = 0; i < this->device_count; i++)
  {
    WriteLogDevice* device = this->devices + i;

    if (device->addr.interf == PLAYER_SONAR_CODE)
    {
      // Get the sonar geometry
      Message* msg;
      if(!(msg = device->device->Request(this->InQueue,
                                         PLAYER_MSGTYPE_REQ,
                                         PLAYER_SONAR_REQ_GET_GEOM,
                                         NULL, 0, NULL, true)))
      {
        // oh well.
        PLAYER_WARN("unable to get sonar geometry");
      }
      else
      {
        // log it
        this->Write(device, msg->GetHeader(), msg->GetPayload());
        delete msg;
      }
    }
    else if (device->addr.interf == PLAYER_LASER_CODE)
    {
      // Get the laser geometry
      Message* msg;
      if(!(msg = device->device->Request(this->InQueue,
                                         PLAYER_MSGTYPE_REQ,
                                         PLAYER_LASER_REQ_GET_GEOM,
                                         NULL, 0, NULL, true)))
      {
        // oh well.
        PLAYER_WARN("unable to get laser geometry");
      }
      else
      {
        // log it
        this->Write(device, msg->GetHeader(), msg->GetPayload());
        delete msg;
      }
    }
    else if (device->addr.interf == PLAYER_POSITION2D_CODE)
    {
      // Get the laser geometry
      Message* msg;
      if(!(msg = device->device->Request(this->InQueue,
                                         PLAYER_MSGTYPE_REQ,
                                         PLAYER_POSITION2D_REQ_GET_GEOM,
                                         NULL, 0, NULL, true)))
      {
        // oh well.
        PLAYER_WARN("unable to get position geometry");
      }
      else
      {
        // log it
        this->Write(device, msg->GetHeader(), msg->GetPayload());
        delete msg;
      }
    }
  }
}

void
WriteLog::CloseFile()
{
  if(this->file)
  {
    fflush(this->file);
    fclose(this->file);
    this->file = NULL;
  }
}

int
WriteLog::ProcessMessage(MessageQueue * resp_queue,
                         player_msghdr * hdr,
                         void * data)
{
  if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
                           PLAYER_LOG_REQ_SET_WRITE_STATE,
                           this->device_addr))
  {
    if(hdr->size != sizeof(player_log_set_write_state_t))
    {
      PLAYER_ERROR2("request is wrong length (%d != %d); ignoring",
                    hdr->size, sizeof(player_log_set_write_state_t));
      return(-1);
    }
    player_log_set_write_state_t* sreq = (player_log_set_write_state_t*)data;

    if(sreq->state)
    {
      puts("WriteLog: start logging");
      this->enable = true;
    }
    else
    {
      puts("WriteLog: stop logging");
      this->enable = false;
    }

    // send an empty ACK
    this->Publish(this->device_addr, resp_queue,
                  PLAYER_MSGTYPE_RESP_ACK,
                  PLAYER_LOG_REQ_SET_WRITE_STATE);
    return(0);
  }
  else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
                                PLAYER_LOG_REQ_GET_STATE, this->device_addr))
  {
    if(hdr->size != 0)
    {
      PLAYER_ERROR2("request is wrong length (%d != %d); ignoring",
                    hdr->size, 0);
      return(-1);
    }

    player_log_get_state_t greq;

    greq.type = PLAYER_LOG_TYPE_WRITE;
    if(this->enable)
      greq.state = 1;
    else
      greq.state = 0;

    this->Publish(this->device_addr,
                  resp_queue,
                  PLAYER_MSGTYPE_RESP_ACK,
                  PLAYER_LOG_REQ_GET_STATE,
                  (void*)&greq, sizeof(greq), NULL);
    return(0);
  }
  else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
                                PLAYER_LOG_REQ_SET_FILENAME, this->device_addr))
  {
    if(hdr->size < sizeof(uint32_t))
    {
      PLAYER_ERROR2("request is wrong length (%d < %d); ignoring",
                    hdr->size, sizeof(uint32_t));
      return(-1);
    }
    player_log_set_filename_t* freq = (player_log_set_filename_t*)data;

    if(this->enable)
    {
      PLAYER_WARN("tried to switch filenames while logging");
      return(-1);
    }

    PLAYER_MSG1(1,"Closing logfile %s", this->filename);
    this->CloseFile();
    memset(this->filename,0,sizeof(this->filename));
    strncpy(this->filename,
            (const char*)freq->filename,
            freq->filename_count);
    this->filename[sizeof(this->filename)-1] = '\0';
    PLAYER_MSG1(1,"Opening logfile %s", this->filename);
    if(this->OpenFile() < 0)
    {
      PLAYER_WARN1("Failed to open logfile %s", this->filename);
      return(-1);
    }

    this->Publish(this->device_addr, resp_queue,
                  PLAYER_MSGTYPE_RESP_ACK, PLAYER_LOG_REQ_SET_FILENAME);
    return(0);
  }
  else if(hdr->type == PLAYER_MSGTYPE_DATA)
  {
    // If logging is stopped, then don't log
    if(!this->enable)
      return(0);

    // Walk the device list
    for (int i = 0; i < this->device_count; i++)
    {
      WriteLogDevice * device = this->devices + i;

      if((device->addr.host != hdr->addr.host) ||
         (device->addr.robot != hdr->addr.robot) ||
         (device->addr.interf != hdr->addr.interf) ||
         (device->addr.index != hdr->addr.index))
        continue;


      // Write data to file
      this->Write(device, hdr, data);
      return(0);
    }
    return(-1);
  }
  return(-1);
}

////////////////////////////////////////////////////////////////////////////////
// Main function for device thread
void
WriteLog::Main(void)
{
  if(this->OpenFile() < 0)
  {
    PLAYER_ERROR2("unable to open [%s]: %s\n", this->filename, strerror(errno));
    return;
  }

  while (1)
  {
    pthread_testcancel();

    // Wait on my queue
    this->Wait();

    // Process all new messages (calls ProcessMessage on each)
    this->ProcessMessages();
  }
}


////////////////////////////////////////////////////////////////////////////
// Write data to file
void WriteLog::Write(WriteLogDevice *device,
                     player_msghdr_t* hdr,
                     void *data)
{
  //char host[256];
  player_interface_t iface;

  // Get interface name
  assert(device);
  ::lookup_interface_code(device->addr.interf, &iface);
  //gethostname(host, sizeof(host));

  // Write header info
  fprintf(this->file, "%014.3f %u %u %s %02u %03u %03u ",
          hdr->timestamp,
          device->addr.host,
          device->addr.robot,
          iface.name,
          device->addr.index,
          hdr->type,
          hdr->subtype);


  int retval;
  // Write the data
  switch (iface.interf)
  {
    case PLAYER_LASER_CODE:
      retval = this->WriteLaser(hdr, data);
      break;
    case PLAYER_POSITION2D_CODE:
      retval = this->WritePosition(hdr, data);
      break;
    case PLAYER_PTZ_CODE:
      retval = this->WritePTZ(hdr, data);
      break;
    case PLAYER_SONAR_CODE:
      retval = this->WriteSonar(hdr, data);
      break;
    case PLAYER_WIFI_CODE:
      retval = this->WriteWiFi(hdr, data);
      break;
    case PLAYER_WSN_CODE:
      retval = this->WriteWSN(hdr, data);
      break;
#if 0
    case PLAYER_BLOBFINDER_CODE:
      this->WriteBlobfinder((player_blobfinder_data_t*) data);
      break;
    case PLAYER_CAMERA_CODE:
      this->WriteCamera((player_camera_data_t*) data);
      if (this->cameraSaveImages)
        this->WriteCameraImage(device, (player_camera_data_t*) data, &time);
      break;
    case PLAYER_FIDUCIAL_CODE:
      this->WriteFiducial((player_fiducial_data_t*) data);
      break;
    case PLAYER_GPS_CODE:
      this->WriteGps((player_gps_data_t*) data);
      break;
    case PLAYER_JOYSTICK_CODE:
      this->WriteJoystick((player_joystick_data_t*) data);
      break;
    case PLAYER_POSITION3D_CODE:
      this->WritePosition3d((player_position3d_data_t*) data);
      break;
    case PLAYER_POWER_CODE:
      this->WritePower((player_power_data_t*) data);
      break;
    case PLAYER_PLAYER_CODE:
      break;
#endif
    default:
      PLAYER_WARN1("unsupported interface type [%s]",
                   ::lookup_interface_name(0, iface.interf));
      retval = -1;
      break;
  }

  if(retval < 0)
    PLAYER_WARN2("not logging message to interface \"%s\" with subtype %d",
                 ::lookup_interface_name(0, iface.interf), hdr->subtype);

  fprintf(this->file, "\n");

  // Flush the data (some drivers produce a lot of data; we dont want
  // it to back up and slow us down later).
  fflush(this->file);

  return;
}

/** @ingroup tutorial_datalog
 * @defgroup player_driver_writelog_laser laser format

@brief laser log format

The following type:subtype laser messages can be logged:
- 1:1 (PLAYER_LASER_DATA_SCAN) - A scan.  The format is:
  - scan_id (int): unique, usually increasing index associated with the scan
  - min_angle (float): minimum scan angle, in radians
  - max_angle (float): maximum scan angle, in radians
  - resolution (float): angular resolution, in radians
  - max_range (float): maximum scan range, in meters
  - count (int): number of readings to follow
  - list of readings; for each reading:
    - range (float): in meters
    - intensity (int): intensity

- 1:2 (PLAYER_LASER_DATA_SCANPOSE) - A scan with an attached pose. The format is:
  - scan_id (int): unique, usually increasing index associated with the scan
  - px (float): X coordinate of the pose of the laser's parent object (e.g., the robot to which it is attached), in meters
  - py (float): Y coordinate of the pose of the laser's parent object (e.g., the robot to which it is attached), in meters
  - pa (float): yaw coordinate of the pose of the laser's parent object (e.g., the robot to which it is attached), in radians
  - min_angle (float): minimum scan angle, in radians
  - max_angle (float): maximum scan angle, in radians
  - resolution (float): angular resolution, in radians
  - max_range (float): maximum scan range, in meters
  - count (int): number of readings to follow
  - list of readings; for each reading:
    - range (float): in meters
    - intensity (int): intensity

- 4:1 (PLAYER_LASER_REQ_GET_GEOM) - Laser pose information. The format is:
  - lx (float): X coordinate of the laser's pose wrt its parent (e.g., the robot to which it is attached), in meters.
  - ly (float): Y coordinate of the laser's pose wrt its parent (e.g., the robot to which it is attached), in meters.
  - la (float): yaw coordinate of the laser's pose wrt its parent (e.g., the robot to which it is attached), in radians.
  - sx (float): length of the laser
  - sy (float): width of the laser
*/
int
WriteLog::WriteLaser(player_msghdr_t* hdr, void *data)
{
  size_t i;
  player_laser_data_t* scan;
  player_laser_data_scanpose_t* scanpose;
  player_laser_geom_t* geom;

  // Check the type
  switch(hdr->type)
  {
    case PLAYER_MSGTYPE_DATA:
      // Check the subtype
      switch(hdr->subtype)
      {
        case PLAYER_LASER_DATA_SCAN:
          scan = (player_laser_data_t*)data;
          // Note that, in this format, we need a lot of precision in the
          // resolution field.

          fprintf(this->file, "%04d %+07.4f %+07.4f %+.8f %+07.4f %04d ",
                  scan->id, scan->min_angle, scan->max_angle,
                  scan->resolution, scan->max_range, scan->ranges_count);

          for (i = 0; i < scan->ranges_count; i++)
            fprintf(this->file, "%.3f %2d ",
                    scan->ranges[i], scan->intensity[i]);
          return(0);

        case PLAYER_LASER_DATA_SCANPOSE:
          scanpose = (player_laser_data_scanpose_t*)data;
          // Note that, in this format, we need a lot of precision in the
          // resolution field.

          fprintf(this->file, "%04d %+07.3f %+07.3f %+07.3f %+07.4f %+07.4f %+.8f %+07.4f %04d ",
                  scanpose->scan.id,
                  scanpose->pose.px, scanpose->pose.py, scanpose->pose.pa,
                  scanpose->scan.min_angle, scanpose->scan.max_angle,
                  scanpose->scan.resolution, scanpose->scan.max_range,
                  scanpose->scan.ranges_count);

          for (i = 0; i < scanpose->scan.ranges_count; i++)
            fprintf(this->file, "%.3f %2d ",
                    scanpose->scan.ranges[i], scanpose->scan.intensity[i]);
          return(0);

        default:
          return(-1);
      }
    case PLAYER_MSGTYPE_RESP_ACK:
      switch(hdr->subtype)
      {
        case PLAYER_LASER_REQ_GET_GEOM:
          geom = (player_laser_geom_t*)data;
          fprintf(this->file, "%+7.3f %+7.3f %7.3f %7.3f %7.3f",
                  geom->pose.px,
                  geom->pose.py,
                  geom->pose.pa,
                  geom->size.sl,
                  geom->size.sw);
          return(0);
        default:
          return(-1);
      }
    default:
      return(-1);
  }
}

/** @ingroup tutorial_datalog
 * @defgroup player_driver_writelog_position position2d format

@brief position2d log format

The following type:subtype position2d messages can be logged:
- 1:1 (PLAYER_POSITION2D_DATA_STATE) Odometry information.  The format is:
  - px (float): X coordinate of the device's pose, in meters
  - py (float): Y coordinate of the device's pose, in meters
  - pa (float): yaw coordinate of the device's pose, in radians
  - vx (float): X coordinate of the device's velocity, in meters/sec
  - vy (float): Y coordinate of the device's velocity, in meters/sec
  - va (float): yaw coordinate of the device's velocity, in radians/sec
  - stall (int): Motor stall flag

- 4:0 (PLAYER_POSITION2D_REQ_GET_GEOM) Geometry info.  The format is:
  - px (float): X coordinate of the offset of the device's center of rotation, in meters
  - py (float): Y coordinate of the offset of the device's center of rotation, in meters
  - pa (float): yaw coordinate of the offset of the device's center of rotation, in radians
  - sx (float): The device's length, in meters
  - sy (float): The device's width, in meters
*/
int
WriteLog::WritePosition(player_msghdr_t* hdr, void *data)
{
  // Check the type
  switch(hdr->type)
  {
    case PLAYER_MSGTYPE_DATA:
      // Check the subtype
      switch(hdr->subtype)
      {
        case PLAYER_POSITION2D_DATA_STATE:
          {
            player_position2d_data_t* pdata =
                    (player_position2d_data_t*)data;
            fprintf(this->file,
                    "%+07.3f %+07.3f %+04.3f %+07.3f %+07.3f %+07.3f %d",
                    pdata->pos.px,
                    pdata->pos.py,
                    pdata->pos.pa,
                    pdata->vel.px,
                    pdata->vel.py,
                    pdata->vel.pa,
                    pdata->stall);
            return(0);
          }
        default:
          return(-1);
      }

    case PLAYER_MSGTYPE_RESP_ACK:
      // Check the subtype
      switch(hdr->subtype)
      {
        case PLAYER_POSITION2D_REQ_GET_GEOM:
          {
            player_position2d_geom_t* gdata =
                    (player_position2d_geom_t*)data;
            fprintf(this->file,
                    "%+07.3f %+07.3f %+04.3f %+07.3f %+07.3f",
                    gdata->pose.px,
                    gdata->pose.py,
                    gdata->pose.pa,
                    gdata->size.sl,
                    gdata->size.sw);

            return(0);
          }
        default:
          return(-1);
      }

    default:
      return(-1);
  }
}

/** @ingroup tutorial_datalog
 @defgroup player_driver_writelog_ptz ptz format
 
@brief PTZ log format
The format for each @ref interface_wsn message is:
  - pan       (float): The pan angle/value
  - tilt      (float): The tilt angle/value
  - zoom      (float): The zoom factor
  - panspeed  (float): The current panning speed
  - tiltspeed (float): The current tilting speed
 */
int
WriteLog::WritePTZ (player_msghdr_t* hdr, void *data)
{
  // Check the type
  switch(hdr->type)
  {
    case PLAYER_MSGTYPE_DATA:
      // Check the subtype
      switch(hdr->subtype)
      {
        case PLAYER_PTZ_DATA_STATE:
          {
            player_ptz_data_t* pdata =
                    (player_ptz_data_t*)data;
            fprintf(this->file,
                    "%+07.3f %+07.3f %+04.3f %+07.3f %+07.3f",
                    pdata->pan,
                    pdata->tilt,
                    pdata->zoom,
                    pdata->panspeed,
                    pdata->tiltspeed);
            return(0);
          }
        default:
          return(-1);
      }
    default:
      return(-1);
  }
}

/** @ingroup tutorial_datalog
 @defgroup player_driver_writelog_sonar sonar format

@brief sonar log format

The following type:subtype sonar messages can be logged:
- 1:1 (PLAYER_SONAR_DATA_RANGES) Range data.  The format is:
  - range_count (int): number range values to follow
  - list of readings; for each reading:
    - range (float): in meters

- 1:2 (PLAYER_SONAR_DATA_GEOM) Geometry data. The format is:
  - pose_count (int): number of sonar poses to follow
  - list of tranducer poses; for each pose:
    - x (float): relative X position of transducer, in meters
    - y (float): relative Y position of transducer, in meters
    - a (float): relative yaw orientation of transducer, in radians

- 4:1 (PLAYER_SONAR_REQ_GET_GEOM) Geometry info. The format is:
  - pose_count (int): number of sonar poses to follow
  - list of tranducer poses; for each pose:
    - x (float): relative X position of transducer, in meters
    - y (float): relative Y position of transducer, in meters
    - a (float): relative yaw orientation of transducer, in radians
*/
int
WriteLog::WriteSonar(player_msghdr_t* hdr, void *data)
{
  unsigned int i;
  player_sonar_geom_t* geom;
  player_sonar_data_t* range_data;

  // Check the type
  switch(hdr->type)
  {
    case PLAYER_MSGTYPE_DATA:

      // Check the subtype
      switch(hdr->subtype)
      {
        case PLAYER_SONAR_DATA_GEOM:
          // Format:
          //   pose_count x0 y0 a0 x1 y1 a1 ...
          geom = (player_sonar_geom_t*)data;
          fprintf(this->file, "%u ", geom->poses_count);
          for(i=0;i<geom->poses_count;i++)
            fprintf(this->file, "%+07.3f %+07.3f %+07.4f ",
                    geom->poses[i].px,
                    geom->poses[i].py,
                    geom->poses[i].pa);
          return(0);

        case PLAYER_SONAR_DATA_RANGES:
          // Format:
          //   range_count r0 r1 ...
          range_data = (player_sonar_data_t*)data;
          fprintf(this->file, "%u ", range_data->ranges_count);
          for(i=0;i<range_data->ranges_count;i++)
            fprintf(this->file, "%.3f ", range_data->ranges[i]);

          return(0);
        default:
          return(-1);
      }

    case PLAYER_MSGTYPE_RESP_ACK:
      switch(hdr->subtype)
      {
        case PLAYER_SONAR_REQ_GET_GEOM:
          // Format:
          //   pose_count x0 y0 a0 x1 y1 a1 ...
          geom = (player_sonar_geom_t*)data;
          fprintf(this->file, "%u ", geom->poses_count);
          for(i=0;i<geom->poses_count;i++)
            fprintf(this->file, "%+07.3f %+07.3f %+07.4f ",
                    geom->poses[i].px,
                    geom->poses[i].py,
                    geom->poses[i].pa);

          return(0);
        default:
          return(-1);
      }
    default:
      return(-1);
  }
}

/** @ingroup tutorial_datalog
 * @defgroup player_driver_writelog_wifi WiFi format

@brief wifi log format

The format for each @ref interface_wifi message is:
  - links_count (int): number of nodes to follow
  - list of nodes; for each node:
    - mac (string): MAC address
    - ip (string): IP address
    - essid (string): ESSID
    - mode (int): operating mode (master, adhoc, etc.)
    - freq (int): in MHz
    - encrypt (int): encrypted?
    - qual (int): link quality
    - level (int): link level
    - noise (int): noise level
*/
int
WriteLog::WriteWiFi(player_msghdr_t* hdr, void *data)
{
  unsigned int i;
  char mac[32], ip[32], essid[32];
  player_wifi_data_t* wdata;

  // Check the type
  switch(hdr->type)
  {
    case PLAYER_MSGTYPE_DATA:
      // Check the subtype
      switch(hdr->subtype)
      {
	case PLAYER_WIFI_DATA_STATE:
	  wdata = (player_wifi_data_t*)data;
          fprintf(this->file, "%04d ", wdata->links_count);

          for (i = 0; i < wdata->links_count; i++)
          {
	    memset(mac,0,sizeof(mac));
	    memset(ip,0,sizeof(ip));
	    memset(essid,0,sizeof(essid));

	    assert(wdata->links[i].mac_count < sizeof(mac));
	    assert(wdata->links[i].ip_count < sizeof(ip));
	    assert(wdata->links[i].essid_count < sizeof(essid));

	    memcpy(mac, wdata->links[i].mac, wdata->links[i].mac_count);
	    memcpy(ip, wdata->links[i].ip, wdata->links[i].ip_count);
            memcpy(essid, wdata->links[i].essid, wdata->links[i].essid_count);
        
            fprintf(this->file, "'%s' '%s' '%s' %d %d %d %d %d %d ",
                    mac, ip, essid,
                    wdata->links[i].mode, 
                    wdata->links[i].freq,
                    wdata->links[i].encrypt,
                    wdata->links[i].qual,
                    wdata->links[i].level,
                    wdata->links[i].noise);
          }
          return(0);

        default:
          return(-1);
      }

   default:
     return(-1);
  }
}

/** @ingroup tutorial_datalog
 * @defgroup player_driver_writelog_wsn WSN format

@brief WSN log format

The format for each @ref interface_wsn message is:
  - node_type      (int): The type of WSN node
  - node_id        (int): The ID of the WSN node
  - node_parent_id (int): The ID of the WSN node's parent (if existing)
  - data_packet         : The WSN node's data packet
    - light       (float): light measurement from a light sensor
    - mic         (float): accoustic measurement from a microphone
    - accel_x     (float): acceleration on X-axis from an acceleration sensor
    - accel_y     (float): acceleration on Y-axis from an acceleration sensor
    - accel_z     (float): acceleration on Z-axis from an acceleration sensor
    - magn_x      (float): magnetic measurement on X-axis from a magnetometer
    - magn_y      (float): magnetic measurement on Y-axis from a magnetometer
    - magn_z      (float): magnetic measurement on Z-axis from a magnetometer
    - temperature (float): temperature measurement from a temperature sensor
    - battery     (float): remaining battery voltage
 */
int
WriteLog::WriteWSN(player_msghdr_t* hdr, void *data)
{
    //unsigned int i;
    player_wsn_data_t* wdata;

  // Check the type
    switch(hdr->type)
    {
        case PLAYER_MSGTYPE_DATA:
      // Check the subtype
            switch(hdr->subtype)
            {
                case PLAYER_WSN_DATA:
                    wdata = (player_wsn_data_t*)data;
                    fprintf(this->file,"%d %d %d %f %f %f %f %f %f %f %f %f %f",
                            wdata->node_type, 
                            wdata->node_id, 
                            wdata->node_parent_id,
                            wdata->data_packet.light,
                            wdata->data_packet.mic,
                            wdata->data_packet.accel_x,
                            wdata->data_packet.accel_y,
                            wdata->data_packet.accel_z,
                            wdata->data_packet.magn_x,
                            wdata->data_packet.magn_y,
                            wdata->data_packet.magn_z,
                            wdata->data_packet.temperature,
                            wdata->data_packet.battery);
                    return(0);

                default:
                    return(-1);
            }

        default:
            return(-1);
    }
}

#if 0
/** @ingroup tutorial_datalog
 * @defgroup player_driver_writelog_blobfinder Blobfinder format

@brief blobfinder log format

The format for each @ref interface_blobfinder message is:
  - width (int): in pixels, of image
  - height (int): in pixels, of image
  - count (int): number of blobs to follow
  - list of blobs; for each blob:
    - id (int): id of blob (if supported)
    - color (int): packed 24-bit RGB color of blob
    - area (int): in pixels, of blob
    - x y (ints): in pixels, of blob center
    - left right top bottom (ints): in pixels, of bounding box
    - range (int): in mm, of range to blob (if supported)

*/
void WriteLog::WriteBlobfinder(player_blobfinder_data_t *data)
{

  fprintf(this->file, " %d %d %d",
          HUINT16(data->width),
          HUINT16(data->height),
          HUINT16(data->blob_count));

  for(int i=0; i < HUINT16(data->blob_count); i++)
  {
    fprintf(this->file, " %d %d %d %d %d %d %d %d %d %f",
            HINT16(data->blobs[i].id),
            HUINT32(data->blobs[i].color),
            HUINT32(data->blobs[i].area),
            HUINT16(data->blobs[i].x),
            HUINT16(data->blobs[i].y),
            HUINT16(data->blobs[i].left),
            HUINT16(data->blobs[i].right),
            HUINT16(data->blobs[i].top),
            HUINT16(data->blobs[i].bottom),
            MM_M(HUINT16(data->blobs[i].range)));
  }

  return;
}


/** @ingroup tutorial_datalog
 * @defgroup player_driver_writelog_camera Camera format

@brief camera data format

The format for each @ref interface_camera message is:
  - width (int): in pixels
  - height (int): in pixels
  - depth (int): in bits per pixel
  - format (int): image format
  - compression (int): image compression
  - image data, encoded as a string of ASCII hex values
*/
void WriteLog::WriteCamera(player_camera_data_t *data)
{
  char *str;
  size_t src_size, dst_size;

  // Image format
  fprintf(this->file, "%d %d %d %d %d %d " ,
          HUINT16(data->width), HUINT16(data->height),
          data->bpp, data->format, data->compression,
          HUINT32(data->image_size));

  // Check image size
  src_size = HUINT32(data->image_size);
  dst_size = ::EncodeHexSize(src_size);
  str = (char*) malloc(dst_size + 1);

  // Encode image into string
  ::EncodeHex(str, dst_size, data->image, src_size);

  // Write image bytes
  fprintf(this->file, str);
  free(str);

  return;
}


////////////////////////////////////////////////////////////////////////////
// Write camera data to image file as well
void WriteLog::WriteCameraImage(WriteLogDevice *device, player_camera_data_t *data, struct timeval *time)
{
  FILE *file;
  char filename[1024];

  if (data->compression != PLAYER_CAMERA_COMPRESS_RAW)
  {
    PLAYER_WARN("unsupported compression method");
    return;
  }

  snprintf(filename, sizeof(filename), "%s_camera_%02d_%06d.pnm",
           this->default_basename, device->id.index, device->cameraFrame++);

  file = fopen(filename, "w+");
  if (file == NULL)
    return;

  if (data->format == PLAYER_CAMERA_FORMAT_RGB888)
  {
    // Write ppm header
    fprintf(file, "P6\n%d %d\n%d\n", HUINT16(data->width), HUINT16(data->height), 255);
    fwrite(data->image, 1, HUINT32(data->image_size), file);
  }
  else if (data->format == PLAYER_CAMERA_FORMAT_MONO8)
  {
    // Write pgm header
    fprintf(file, "P5\n%d %d\n%d\n", HUINT16(data->width), HUINT16(data->height), 255);
    fwrite(data->image, 1, HUINT32(data->image_size), file);
  }
  else
  {
    PLAYER_WARN("unsupported image format");
  }

  fclose(file);

  return;
}


/** @ingroup tutorial_datalog
 * @defgroup player_driver_writelog_fiducial Fiducial format

@brief fiducial log format

The format for each @ref interface_fiducial message is:
  - count (int): number of fiducials to follow
  - list of fiducials; for each fiducial:
    - id (int): fiducial ID
    - x (float): relative X position, in meters
    - y (float): relative Y position, in meters
    - z (float): relative Z position, in meters
    - roll (float): relative roll orientation, in radians
    - pitch (float): relative pitch orientation, in radians
    - yaw (float): relative yaw orientation, in radians
    - ux (float): uncertainty in relative X position, in meters
    - uy (float): uncertainty in relative Y position, in meters
    - uz (float): uncertainty in relative Z position, in meters
    - uroll (float): uncertainty in relative roll orientation, in radians
    - upitch (float): uncertainty in relative pitch orientation, in radians
    - uyaw (float): uncertainty in relative yaw orientation, in radians

*/
void WriteLog::WriteFiducial(player_fiducial_data_t *data)
{
  // format: <count> [<id> <x> <y> <z> <roll> <pitch> <yaw> <ux> <uy> <uz> etc] ...
  fprintf(this->file, "%d", HUINT16(data->count));
  for(int i=0;i<HUINT16(data->count);i++)
  {
    fprintf(this->file, " %d"
            " %+07.3f %+07.3f %+07.3f %+07.3f %+07.3f %+07.3f"
            " %+07.3f %+07.3f %+07.3f %+07.3f %+07.3f %+07.3f",
            HINT16(data->fiducials[i].id),
            MM_M(HINT32(data->fiducials[i].pos[0])),
            MM_M(HINT32(data->fiducials[i].pos[1])),
            MM_M(HINT32(data->fiducials[i].pos[2])),
            MM_M(HINT32(data->fiducials[i].rot[0])),
            MM_M(HINT32(data->fiducials[i].rot[1])),
            MM_M(HINT32(data->fiducials[i].rot[2])),
            MM_M(HINT32(data->fiducials[i].upos[0])),
            MM_M(HINT32(data->fiducials[i].upos[1])),
            MM_M(HINT32(data->fiducials[i].upos[2])),
            MM_M(HINT32(data->fiducials[i].urot[0])),
            MM_M(HINT32(data->fiducials[i].urot[1])),
            MM_M(HINT32(data->fiducials[i].urot[2])));
  }

  return;
}


/** @ingroup tutorial_datalog
 * @defgroup player_driver_writelog_gps GPS format

@brief gps log format

The format for each @ref interface_gps message is:
  - time (float): current GPS time, in seconds
  - latitude (float): in degrees
  - longitude (float): in degrees
  - altitude (float): in meters
  - utm_e (float): UTM WGS84 easting, in meters
  - utm_n (float): UTM WGS84 northing, in meters
  - hdop (float): horizontal dilution of position
  - vdop (float): vertical dilution of position
  - err_horz (float): horizontal error, in meters
  - err_vert (float): vertical error, in meters
  - quality (int): quality of fix (0 = invalid, 1 = GPS fix, 2 = DGPS fix)
  - num_sats (int): number of satellites in view
*/
void WriteLog::WriteGps(player_gps_data_t *data)
{
  fprintf(this->file,
          "%.3f "
          "%.6f %.6f %.6f "
          "%.3f %.3f "
          "%.3f %.3f %.3f %.3f "
          "%d %d",
          (double) (uint32_t) HINT32(data->time_sec) +
          (double) (uint32_t) HINT32(data->time_sec) * 1e-6,
          (double) HINT32(data->latitude) / (1e7),
          (double) HINT32(data->longitude) / (1e7),
          MM_M(HINT32(data->altitude)),
          CM_M(HINT32(data->utm_e)),
          CM_M(HINT32(data->utm_n)),
          (double) HINT16(data->hdop) / 10,
          (double) HINT16(data->vdop) / 10,
          MM_M(HINT32(data->err_horz)),
          MM_M(HINT32(data->err_vert)),
          (int) data->quality,
          (int) data->num_sats);

  return;
}


/** @ingroup tutorial_datalog
 * @defgroup player_driver_writelog_joystick Joystick format

@brief joystick log format

The format for each @ref interface_joystick message is:
  - xpos (int): unscaled X position of joystick
  - ypos (int): unscaled Y position of joystick
  - xscale (int): maximum X position
  - yscale (int): maximum Y position
  - buttons (hex string): bitmask of button states
*/
void WriteLog::WriteJoystick(player_joystick_data_t *data)
{
  fprintf(this->file, "%+d %+d %d %d %X",
          HINT16(data->xpos),
          HINT16(data->ypos),
          HINT16(data->xscale),
          HINT16(data->yscale),
          HUINT16(data->buttons));

  return;
}



/** @ingroup tutorial_datalog
 * @defgroup player_driver_writelog_position3d Position3d format

@brief position3d format

The format for each @ref interface_position3d message is:
  - xpos (float): in meters
  - ypos (float): in meters
  - zpos (float): in meters
  - roll (float): in radians
  - pitch (float): in radians
  - yaw (float): in radians
  - xspeed (float): in meters / second
  - yspeed (float): in meters / second
  - zspeed (float): in meters / second
  - rollspeed(float): in radians / second
  - pitchspeed(float): in radians / second
  - yawspeed(float): in radians / second
  - stall (int): motor stall sensor
*/
void WriteLog::WritePosition3d(player_position3d_data_t *data)
{
  fprintf(this->file,
          "%+.4f %+.4f %+.4f "
          "%+.4f %+.4f %+.4f "
          "%+.4f %+.4f %+.4f "
          "%+.4f %+.4f %+.4f "
          "%d",
          MM_M(HINT32(data->xpos)),
          MM_M(HINT32(data->ypos)),
          MM_M(HINT32(data->zpos)),
          HINT32(data->roll) / 1000.0,
          HINT32(data->pitch) / 1000.0,
          HINT32(data->yaw) / 1000.0,
          MM_M(HINT32(data->xspeed)),
          MM_M(HINT32(data->yspeed)),
          MM_M(HINT32(data->zspeed)),
          HINT32(data->rollspeed) / 1000.0,
          HINT32(data->pitchspeed) / 1000.0,
          HINT32(data->yawspeed) / 1000.0,
          data->stall);

  return;
}


/** @ingroup tutorial_datalog
 * @defgroup player_driver_writelog_power Power format

@brief power log format

The format for each @ref interface_power message is:
  - charge (float): in volts
*/
void WriteLog::WritePower(player_power_data_t *data)
{
  fprintf(this->file, "%.1f ", HUINT16(data->charge) / 10.0);
  return;
}


#endif


