/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) Andrew Howard 2002-2003
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
 *  Player - One Hell of a Robot Server
 *  Copyright (C) Andrew Howard 2003
 *                      
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
/***************************************************************************
 * Desc: Common device functions
 * Author: Andrew Howard
 * Date: 13 May 2002
 * CVS: $Id: device.c,v 1.11 2005/08/24 21:42:43 gerkey Exp $
 **************************************************************************/

#include <stdlib.h>
#include <string.h>

#include "playerc.h"
#include "error.h"


void playerc_device_init(playerc_device_t *device, playerc_client_t *client,
                         int code, int index, playerc_putmsg_fn_t putmsg)
{
  device->id = device;
  device->client = client;
  device->addr.host = 0;
  device->addr.robot = client->port;
  device->addr.interf = code;
  device->addr.index = index;
  device->subscribed = 0;
  device->callback_count = 0;
  device->putmsg = putmsg;

  if (device->client)
    playerc_client_adddevice(device->client, device);
  return;
}


// Finalize the device
void playerc_device_term(playerc_device_t *device)
{
  if (device->client)
    playerc_client_deldevice(device->client, device);
  return;
}


// Subscribe/unsubscribe the device
int playerc_device_subscribe(playerc_device_t *device, int access)
{
  if (playerc_client_subscribe(device->client, device->addr.interf, 
                               device->addr.index, access, 
                               device->drivername, sizeof(device->drivername)) != 0)
    return -1;
  device->subscribed = 1;
  return 0;
}



// Subscribe/unsubscribe the device
int playerc_device_unsubscribe(playerc_device_t *device)
{
  device->subscribed = 0;
  return playerc_client_unsubscribe(device->client, 
                                    device->addr.interf, 
                                    device->addr.index);
}

