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
///////////////////////////////////////////////////////////////////////////
//
// Desc: Driver for detecting retro-reflective markers in a laser scan.
// Author: Andrew Howard
// Date: 16 Aug 2002
// CVS: $Id: laserbar.cc,v 1.12.2.1 2006/06/07 16:12:48 gerkey Exp $
//
// Theory of operation
//
// Parses a laser scan to find retro-reflective markers.  Currently only
// cylindrical markers are supported.
//
///////////////////////////////////////////////////////////////////////////

/** @ingroup drivers */
/** @{ */
/** @defgroup driver_laserbar laserbar
 * @brief Laser bar detector.
 
The laser bar detector searches for retro-reflective targets in the
laser range finder data.  Targets can be either planar or cylindrical,
as shown below. For planar targets, the range, bearing and orientation
will be determined; for cylindrical targets, only the range and bearing
will be determined.  The target size and shape can be set in the
configuration file.

The range at which targets can be detected is dependant on the target
size, the angular resolution of the laser and the quality of the
retro-reflective material used on the target.

See also the @ref driver_laserbarcode and 
@ref driver_laservisualbarcode drivers.

@image html laservisualbeacon.jpg "A sample laser bar (ignore the colored bands)"

@par Compile-time dependencies

- none

@par Provides

- This driver provides detected target information through a @ref
  interface_fiducial device.

@par Requires

- This driver finds targets in scans from a @ref interface_laser
  device.

@par Configuration requests

- PLAYER_FIDUCIAL_GET_GEOM

@par Configuration file options

- width (length)
  - Default: 0.08 m
  - Target width.

- tol (length)
  - Default: 0.5 m
  - Tolerance.

@par Example

@verbatim
driver
(
  name "laserbar"
  requires ["laser:0"]
  provides ["fiducial:0"]
  width 0.2
)
@endverbatim

@author Andrew Howard
*/
/** @} */

#include "player.h"

#include <errno.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>       // for atoi(3)
#include <netinet/in.h>   // for htons(3)
#include <unistd.h>

#define PLAYER_ENABLE_TRACE 0
#define PLAYER_ENABLE_MSG 0

#include <libplayercore/playercore.h>

/*#include "error.h"
#include "driver.h"
#include "devicetable.h"
#include "drivertable.h"
#include "clientdata.h"
*/

// Driver for detecting laser retro-reflectors.
class LaserBar : public Driver
{
  // Constructor
  public: LaserBar( ConfigFile* cf, int section);

  // Setup/shutdown routines.
  public: virtual int Setup();
  public: virtual int Shutdown();

  // Process incoming messages from clients 
  int ProcessMessage(MessageQueue *resp_queue, player_msghdr *hdr, void *data);

  // Main function for device thread.
  private: virtual void Main();

  // Data configuration.
  /*public: virtual size_t GetData(player_device_id_t id,
                                 void* dest, size_t len,
                                 struct timeval* timestamp);
  public: virtual int PutConfig(player_device_id_t id, void *client, 
                                void *src, size_t len,
                                struct timeval* timestamp);*/

  // Handle geometry requests.
  //private: void HandleGetGeom(void *client, void *request, int len);

  // Analyze the laser data and pick out reflectors.
  private: void Find();

  // Test a patch to see if it has valid moments.
  private: bool TestMoments(double mn, double mr, double mb, double mrr, double mbb);

  // Find the line of best fit for the given segment of the laser
  // scan.  Fills in the pose and pose uncertainty of the reflector
  // (range, bearing, orientation).
  private: void FitCircle(int first, int last,
                          double *pr, double *pb, double *po,
                          double *ur, double *ub, double *uo);

  // Add a item into the fiducial list.
  private: void Add(double pr, double pb, double po,
                    double ur, double ub, double uo);
  
  // Pointer to laser to get data from.
  private:
  Device *laser_device;
  player_devaddr_t laser_addr;

  // Reflector properties.
  private: double reflector_width;
  private: double reflector_tol;
  
  // Local copy of the current laser data.
  private: player_laser_data_t ldata;

  // Local copy of the current fiducial data.
  private: struct timeval ftimestamp;
  private: player_fiducial_data_t fdata;
};


// Initialization function
Driver* LaserBar_Init( ConfigFile* cf, int section)
{
  return ((Driver*) (new LaserBar( cf, section)));
}


// a driver registration function
void LaserBar_Register(DriverTable* table)
{
  table->AddDriver("laserbar", LaserBar_Init);
}


////////////////////////////////////////////////////////////////////////////////
// Constructor
LaserBar::LaserBar( ConfigFile* cf, int section)
  : Driver(cf, section, true, PLAYER_MSGQUEUE_DEFAULT_MAXLEN, PLAYER_FIDUCIAL_CODE) 
{

  // Must have an input laser
  if (cf->ReadDeviceAddr(&this->laser_addr, section, "requires",
                       PLAYER_LASER_CODE, -1, NULL) != 0)
  {
    this->SetError(-1);    
    return;
  }

  // Default reflector properties.
  this->reflector_width = cf->ReadLength(section, "width", 0.08);
  this->reflector_tol = cf->ReadLength(section, "tol", 0.50);
}


////////////////////////////////////////////////////////////////////////////////
// Set up the device (called by server thread).
int LaserBar::Setup()
{

  this->laser_device = deviceTable->GetDevice(this->laser_addr);

  if (!this->laser_device)
  {
    PLAYER_ERROR("unable to locate suitable laser device");
    return(-1);
  }

  if (this->laser_device->Subscribe(this->InQueue) != 0)
  {
    PLAYER_ERROR("unable to subscribe to laser device");
    return -1;
  }
    
  // Subscribe to the laser device, but fail if it fails
/*  if (BaseClient->Subscribe(this->laser_id) != 0)
  {
    PLAYER_ERROR("unable to subscribe to laser device");
    return(-1);
  }*/

  return 0;
}


////////////////////////////////////////////////////////////////////////////////
// Shutdown the device (called by server thread).
int LaserBar::Shutdown()
{
  this->laser_device->Unsubscribe(this->InQueue);
  this->laser_device = NULL;

  return 0;
}


////////////////////////////////////////////////////////////////////////////////
// Main function for device thread
void LaserBar::Main() 
{
  while (true)
  {
    // Let the laser drive update rate
    this->Wait();

    // Test if we are supposed to cancel this thread.
    pthread_testcancel();

    // Process any pending requests.
    this->ProcessMessages();

    //usleep(100000);
  }

  return;
}



////////////////////////////////////////////////////////////////////////////////
// Process an incoming message
int LaserBar::ProcessMessage(MessageQueue *resp_queue, player_msghdr *hdr, void *data)
{

  if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_DATA, 
                            PLAYER_LASER_DATA_SCAN, this->laser_addr))
  {
  	//assert(hdr->size == sizeof(player_laser_data_t));
  	player_laser_data_t *laser_data = reinterpret_cast<player_laser_data_t * > (data);

  	this->Lock();

    this->ldata.min_angle = laser_data->min_angle;
    this->ldata.max_angle = laser_data->max_angle;
    this->ldata.resolution = laser_data->resolution;
    this->ldata.max_range = laser_data->max_range;
    this->ldata.ranges_count = laser_data->ranges_count;
    this->ldata.intensity_count = laser_data->intensity_count;

    for (unsigned int i = 0; i < laser_data->ranges_count; i++)
    {
      this->ldata.ranges[i] = laser_data->ranges[i];
      this->ldata.intensity[i] = laser_data->intensity[i];
    }

    // Analyse the laser data
    this->Find();


  	this->Unlock();

    uint size = sizeof(this->fdata) - sizeof(this->fdata.fiducials) +
      this->fdata.fiducials_count * sizeof(this->fdata.fiducials[0]);

    printf("Count[%d]\n",this->fdata.fiducials_count);

    this->Publish(this->device_addr, NULL, 
                  PLAYER_MSGTYPE_DATA, PLAYER_FIDUCIAL_DATA_SCAN, 
                  reinterpret_cast<void*>(&this->fdata), 
                  size, &hdr->timestamp);


  	return 0;
  }
 
  if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ, 
                            PLAYER_FIDUCIAL_REQ_GET_GEOM, this->device_addr))
  {
    Message *msg;

    player_fiducial_geom_t fgeom;

    if (!(msg = this->laser_device->Request(this->InQueue, PLAYER_MSGTYPE_REQ, PLAYER_LASER_REQ_GET_GEOM, NULL, 0, NULL, false)))
    {
      PLAYER_WARN("failed to get laer geometry");
      fgeom.pose.px = 0.0;
      fgeom.pose.py = 0.0;
      fgeom.pose.pa = 0.0;
    }
    else
    {
      player_laser_geom_t *lgeom = (player_laser_geom_t*)msg->GetPayload();

      fgeom.pose.px = lgeom->pose.px;
      fgeom.pose.py = lgeom->pose.py;
      fgeom.pose.pa = lgeom->pose.pa;
      fgeom.size.sw = lgeom->size.sw;
      fgeom.size.sl = lgeom->size.sl;
    }

    fgeom.fiducial_size.sw = this->reflector_width;
    fgeom.fiducial_size.sl = this->reflector_width;

    delete msg;

    this->Publish(this->device_addr, resp_queue, 
                  PLAYER_MSGTYPE_RESP_ACK, PLAYER_FIDUCIAL_REQ_GET_GEOM, 
                  (void*)&fgeom, sizeof(fgeom), NULL);

    return 0;
  }

  return -1;
}



////////////////////////////////////////////////////////////////////////////////
// Get data from buffer (called by server thread).
/*size_t 
LaserBar::GetData(player_device_id_t id,
                  void* dest, size_t len,
                  struct timeval* timestamp)
{
  int i;
  size_t laser_size;
  struct timeval laser_timestamp;
  
  // Get the laser data.
  laser_size = this->laser_driver->GetData(this->laser_id, 
                                           (void*) &this->ldata, 
                                           sizeof(this->ldata),
                                           &laser_timestamp);
  assert(laser_size <= sizeof(this->ldata));
  
  // If the laser doesnt have new data, just return a copy of our old
  // data.
  if ((laser_timestamp.tv_sec != this->ftimestamp.tv_sec) || 
      (laser_timestamp.tv_usec != this->ftimestamp.tv_usec))
  {
    // Do some byte swapping
    this->ldata.resolution = ntohs(this->ldata.resolution);
    this->ldata.range_res = ntohs(this->ldata.range_res);
    this->ldata.min_angle = ntohs(this->ldata.min_angle);
    this->ldata.max_angle = ntohs(this->ldata.max_angle);
    this->ldata.range_count = ntohs(this->ldata.range_count);
    for (i = 0; i < this->ldata.range_count; i++)
      this->ldata.ranges[i] = ntohs(this->ldata.ranges[i]);

    // Analyse the laser data
    this->Find();

    // Do some byte-swapping on the fiducial data.
    for (i = 0; i < this->fdata.count; i++)
    {
      this->fdata.fiducials[i].pos[0] = htonl(this->fdata.fiducials[i].pos[0]);
      this->fdata.fiducials[i].pos[1] = htonl(this->fdata.fiducials[i].pos[1]);
      this->fdata.fiducials[i].rot[2] = htonl(this->fdata.fiducials[i].rot[2]);
    }
    this->fdata.count = htons(this->fdata.count);
  }

  // Copy results
  assert(len >= sizeof(this->fdata));
  memcpy(dest, &this->fdata, sizeof(this->fdata));

  // Copy the laser timestamp
  *timestamp = laser_timestamp;

  this->ftimestamp = laser_timestamp;
  
  return (sizeof(this->fdata));
}
*/

////////////////////////////////////////////////////////////////////////////////
// Put configuration in buffer (called by server thread)
/*int 
LaserBar::PutConfig(player_device_id_t id, void *client, 
                    void *src, size_t len,
                    struct timeval* timestamp)
{
 int subtype;

  if (len < 1)
  {
    PLAYER_ERROR("empty requestion; ignoring");
    return 0;
  }
  
  subtype = ((uint8_t*) src)[0];
  switch (subtype)
  {
    case PLAYER_FIDUCIAL_GET_GEOM:
    {
      HandleGetGeom(client, src, len);
      break;
    }
    default:
    {
      if (PutReply(client, PLAYER_MSGTYPE_RESP_NACK,NULL) != 0)
        PLAYER_ERROR("PutReply() failed");
      break;
    }
  }
  
  return (0);
}*/


////////////////////////////////////////////////////////////////////////////////
// Handle geometry requests.
/*void LaserBar::HandleGetGeom(void *client, void *request, int len)
{
  unsigned short reptype;
  struct timeval ts;
  int replen;
  player_laser_geom_t lgeom;
  player_fiducial_geom_t fgeom;
    
  // Get the geometry from the laser
  replen = this->laser_driver->Request(this->laser_id, this, 
                                       request, len, NULL,
                                       &reptype, &lgeom, sizeof(lgeom), &ts);
  if (replen <= 0 || replen != sizeof(lgeom))
  {
    PLAYER_ERROR("unable to get geometry from laser device");
    if (PutReply(client, PLAYER_MSGTYPE_RESP_NACK,NULL) != 0)
      PLAYER_ERROR("PutReply() failed");
  }

  fgeom.pose[0] = lgeom.pose[0];
  fgeom.pose[1] = lgeom.pose[1];
  fgeom.pose[2] = lgeom.pose[2];
  fgeom.size[0] = lgeom.size[0];
  fgeom.size[1] = lgeom.size[1];
  fgeom.fiducial_size[0] = ntohs((int) (this->reflector_width * 1000));
  fgeom.fiducial_size[1] = ntohs((int) (this->reflector_width * 1000));
    
  if (PutReply(client, PLAYER_MSGTYPE_RESP_ACK, &fgeom, sizeof(fgeom),&ts) != 0)
    PLAYER_ERROR("PutReply() failed");

  return;
}
*/

////////////////////////////////////////////////////////////////////////////////
// Analyze the laser data to find reflectors.
void LaserBar::Find()
{
  unsigned int i;
  int h;
  double r, b;
  double mn, mr, mb, mrr, mbb;
  double pr, pb, po;
  double ur, ub, uo;

  // Empty the fiducial list.
  this->fdata.fiducials_count = 0;
  
  // Initialise patch statistics.
  mn = 0.0;
  mr = 0.0;
  mb = 0.0;
  mrr = 0.0;
  mbb = 0.0;
    
  // Look for a candidate patch in scan.
  for (i = 0; i < this->ldata.ranges_count; i++)
  {
    r = (double) (this->ldata.ranges[i]);
    b = (double) (this->ldata.min_angle + i * this->ldata.resolution) * M_PI / 180;
    h = (int) (this->ldata.intensity[i]);

    // If there is a reflection...
    if (h > 0)
    {
      mn += 1;
      mr += r;
      mb += b;
      mrr += r * r;
      mbb += b * b;
    }

    // If there is no reflection and we have a patch...
    else if (mn > 0)
    {
      // Compute the moments of the patch.
      mr /= mn;
      mb /= mn;
      mrr = mrr / mn - mr * mr;
      mbb = mbb / mn - mb * mb;
      
      // Apply tests to see if this is a sensible looking patch.
      if (this->TestMoments(mn, mr, mb, mrr, mbb))
      {
        // Do a best fit to determine the pose of the reflector.
        this->FitCircle(i - (int) mn, i - 1, &pr, &pb, &po, &ur, &ub, &uo);

        // Fill in the fiducial data structure.
        this->Add(pr, pb, po, ur, ub, uo);
      }
      
      mn = 0.0;
      mr = 0.0;
      mb = 0.0;
      mrr = 0.0;
      mbb = 0.0;
    }
  }
  return;
}


////////////////////////////////////////////////////////////////////////////////
// Test a patch to see if it has valid moments.
bool LaserBar::TestMoments(double mn, double mr, double mb, double mrr, double mbb)
{
  double dr, db;
  
  //printf("Testing moments %.0f %f %f %f %f\n", mn, mr, mb, mrr, mbb);

  if (mn < 2.0)
    return false;

  // These are tests for a cylindrical reflector.
  dr = (1 + this->reflector_tol) * this->reflector_width / 2;
  db = (1 + this->reflector_tol) * atan2(this->reflector_width / 2, mr);
  if (mrr > dr * dr)
    return false;
  if (mbb > db * db)
    return false;
  
  return true;
}


////////////////////////////////////////////////////////////////////////////////
// Find the line of best fit for the given segment of the laser scan.
// Fills in the pose and pose uncertainty of the reflector (range,
// bearing, orientation).  This one works for cylindrical fiducials.
void LaserBar::FitCircle(int first, int last,
                               double *pr, double *pb, double *po,
                               double *ur, double *ub, double *uo)
{
  int i;
  double r, b;
  double mn, mr, mb;

  mn = 0.0;
  mr = 1e6;
  mb = 0.0;

  for (i = first; i <= last; i++)
  {
    r = (double) (this->ldata.ranges[i]);
    b = (double) (this->ldata.min_angle + i * this->ldata.resolution) * M_PI / 180;

    if (r < mr)
      mr = r;
    mn += 1.0;
    mb += b;
  }

  mr += this->reflector_width / 2;
  mb /= mn;

  *pr = mr;
  *pb = mb;
  *po = 0.0;

  // TODO: put in proper uncertainty estimates.
  *ur = 0.02;  
  *ub = this->ldata.resolution * M_PI / 180;
  *uo = 1e6;
  
  return;
}


////////////////////////////////////////////////////////////////////////////////
// Add a item into the fiducial list.
void LaserBar::Add(double pr, double pb, double po,
                         double ur, double ub, double uo)

{
  player_fiducial_item_t *fiducial;

  //printf("adding %f %f %f\n", pr, pb, po);
  
  //assert(this->fdata.count < ARRAYSIZE(this->fdata.fiducials));

  fiducial = this->fdata.fiducials + this->fdata.fiducials_count++;
  fiducial->id = (int16_t) -1;

  fiducial->pose.px = pr * cos(pb);
  fiducial->pose.py = pr * sin(pb);
  fiducial->pose.pyaw = po;

  return;
}


