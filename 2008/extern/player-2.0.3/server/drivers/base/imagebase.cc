/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey et al
 *                      gerkey@usc.edu    
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
// Desc: base driver for image processing and transform drivers
// Author: Toby Collett
// Date: 15 Feb 2004
// CVS: $Id: imagebase.cc,v 1.1.2.2 2006/06/07 16:12:38 gerkey Exp $
//
///////////////////////////////////////////////////////////////////////////

#include "imagebase.h"

////////////////////////////////////////////////////////////////////////////////
// Constructor

ImageBase::ImageBase(ConfigFile *cf, int section, bool overwrite_cmds, size_t queue_maxlen, int interf)
	: Driver(cf, section, overwrite_cmds, queue_maxlen, interf)
{
  memset(&this->camera_addr, 0, sizeof(player_devaddr_t));
  
  HaveData = false;

  // Must have an input camera
  if (cf->ReadDeviceAddr(&this->camera_addr, section, "requires",
                       PLAYER_CAMERA_CODE, -1, NULL) != 0)
  {
    this->SetError(-1);    
    return;
  }

}


ImageBase::ImageBase(ConfigFile *cf, int section, bool overwrite_cmds, size_t queue_maxlen)
	: Driver(cf, section, overwrite_cmds, queue_maxlen)
{
  memset(&this->camera_addr, 0, sizeof(player_devaddr_t));
  
  HaveData = false;

  // Must have an input camera
  if (cf->ReadDeviceAddr(&this->camera_addr, section, "requires",
                       PLAYER_CAMERA_CODE, -1, NULL) != 0)
  {
    this->SetError(-1);    
    return;
  }

}


////////////////////////////////////////////////////////////////////////////////
// Set up the device (called by server thread).
int ImageBase::Setup()
{
  // Subscribe to the camera.
  if (Device::MatchDeviceAddress (camera_addr, device_addr))
  {
    PLAYER_ERROR ("attempt to subscribe to self");
    return -1;
  }
  if (!(camera_driver = deviceTable->GetDevice (camera_addr)))
  {
    PLAYER_ERROR ("unable to locate suitable camera device");
    return -1;
  }
  if (camera_driver->Subscribe (InQueue) != 0)
  {
    PLAYER_ERROR ("unable to subscribe to camera device");
    return -1;
  }

  StartThread();

  return 0;
}


////////////////////////////////////////////////////////////////////////////////
// Shutdown the device (called by server thread).
int ImageBase::Shutdown()
{
  StopThread();
	
  camera_driver->Unsubscribe(InQueue);

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Process an incoming message
int ImageBase::ProcessMessage (MessageQueue * resp_queue, player_msghdr * hdr, void * data)
{
  assert(hdr);
  assert(data);
  
  if(Message::MatchMessage (hdr, PLAYER_MSGTYPE_DATA, PLAYER_CAMERA_DATA_STATE, camera_addr))
  {
  	Lock();
  	if (!HaveData)
  	{
 		memcpy(&stored_data, data,hdr->size);
 		HaveData = true;
  	}
 	Unlock();
    return 0;
  }
  return -1;
}

void ImageBase::Main()
{
	for(;;)
	{
		pthread_testcancel();
		
		InQueue->Wait();
		
		ProcessMessages();
		
		Lock();
		if (HaveData)
		{
			Unlock();
			ProcessFrame();
			Lock();
			HaveData = false;
		}
		Unlock();
	}
	
}
