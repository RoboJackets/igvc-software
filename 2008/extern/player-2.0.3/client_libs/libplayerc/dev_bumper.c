/* 
 *  libplayerc : a Player client library
 *  Copyright (C) Andrew Howard 2002-2003
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
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
 * Desc: bumper proxy
 * Author: Toby Collett (based on sonar proxy by Andrew Howard)
 * Date: 13 Feb 2004
 * CVS: $Id: dev_bumper.c,v 1.5 2005/10/05 10:10:12 thjc Exp $
 **************************************************************************/

#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <netinet/in.h>

#include "playerc.h"
#include "error.h"


// Local declarations

// Process incoming data
void playerc_bumper_putmsg(playerc_bumper_t *device, 
                          player_msghdr_t *header,
                          void *data);

// Create a new bumper proxy
playerc_bumper_t *playerc_bumper_create(playerc_client_t *client, int index)
{
  playerc_bumper_t *device;

  device = malloc(sizeof(playerc_bumper_t));
  memset(device, 0, sizeof(playerc_bumper_t));
  playerc_device_init(&device->info, client, PLAYER_BUMPER_CODE, index,
 					  (playerc_putmsg_fn_t) playerc_bumper_putmsg);
    
  return device;
}


// Destroy a bumper proxy
void playerc_bumper_destroy(playerc_bumper_t *device)
{
  playerc_device_term(&device->info);
  free(device);
}


// Subscribe to the bumper device
int playerc_bumper_subscribe(playerc_bumper_t *device, int access)
{
  return playerc_device_subscribe(&device->info, access);
}


// Un-subscribe from the bumper device
int playerc_bumper_unsubscribe(playerc_bumper_t *device)
{
  return playerc_device_unsubscribe(&device->info);
}

// Process incoming data
void playerc_bumper_putmsg(playerc_bumper_t *device, 
                          player_msghdr_t *header,
                          void *data)
{
  int i;
  if((header->type == PLAYER_MSGTYPE_DATA) &&
     (header->subtype == PLAYER_BUMPER_DATA_STATE))
  {
  	player_bumper_data_t * bdata = (player_bumper_data_t *) data;
    device->bumper_count = bdata->bumpers_count;

    // data is array of bytes, either as boolean or coded for bumper corner
    for (i = 0; i < device->bumper_count; i++)
      device->bumpers[i] = bdata->bumpers[i];
  }
  else if((header->type == PLAYER_MSGTYPE_RESP_ACK) &&
     (header->subtype == PLAYER_BUMPER_GET_GEOM))
  {
  	player_bumper_geom_t * bgeom = (player_bumper_geom_t *) data;
  	
    device->pose_count = bgeom->bumper_def_count;
    for (i = 0; i < device->pose_count; i++)
    {
      device->poses[i] = bgeom->bumper_def[i];
    }
  }
}

// Get the bumper geometry.  The writes the result into the proxy
// rather than returning it to the caller.
int playerc_bumper_get_geom(playerc_bumper_t *device)
{
  int i;
  player_bumper_geom_t config;

//  config.subtype = PLAYER_BUMPER_GET_GEOM_REQ;

  if (playerc_client_request(device->info.client, &device->info, PLAYER_BUMPER_GET_GEOM,
                               NULL, &config, sizeof(config)) < 0)
    return -1;
  device->pose_count = config.bumper_def_count;
  for (i = 0; i < device->pose_count; i++)
  {
    device->poses[i] = config.bumper_def[i];
  }
	
  return 0;
}


