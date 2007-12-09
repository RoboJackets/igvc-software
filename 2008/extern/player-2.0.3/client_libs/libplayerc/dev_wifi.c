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
 * Desc: WiFi device proxy
 * Author: Andrew Howard
 * Date: 13 May 2002
 * CVS: $Id: dev_wifi.c,v 1.12.4.1 2006/06/09 01:17:51 gerkey Exp $
 **************************************************************************/

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <netinet/in.h>

#include "playerc.h"
#include "error.h"

// Local declarations
void playerc_wifi_putmsg(playerc_wifi_t *self, player_msghdr_t *header,
                          player_wifi_data_t *data, size_t len);


// Create a new wifi proxy
playerc_wifi_t *playerc_wifi_create(playerc_client_t *client, int index)
{
  playerc_wifi_t *self;

  self = malloc(sizeof(playerc_wifi_t));
  memset(self, 0, sizeof(playerc_wifi_t));
  playerc_device_init(&self->info, client, PLAYER_WIFI_CODE, index,
                      (playerc_putmsg_fn_t) playerc_wifi_putmsg);

  return self;
}


// Destroy a wifi proxy
void playerc_wifi_destroy(playerc_wifi_t *self)
{
  playerc_device_term(&self->info);
  free(self);
}


// Subscribe to the wifi device
int playerc_wifi_subscribe(playerc_wifi_t *self, int access)
{
  return playerc_device_subscribe(&self->info, access);
}


// Un-subscribe from the wifi device
int playerc_wifi_unsubscribe(playerc_wifi_t *self)
{
  return playerc_device_unsubscribe(&self->info);
}


// Process incoming data
void playerc_wifi_putmsg(playerc_wifi_t *self, player_msghdr_t *header,
                          player_wifi_data_t *data, size_t len)
{
  int i;

  if((header->type == PLAYER_MSGTYPE_DATA))
  {
  self->link_count = data->links_count;

  for (i = 0; i < self->link_count; i++)
  {
    strncpy((char*)self->links[i].mac, (char*)data->links[i].mac, sizeof(self->links[i].mac));
    strncpy((char*)self->links[i].ip, (char*)data->links[i].ip, sizeof(self->links[i].ip));
    strncpy((char*)self->links[i].essid, (char*)data->links[i].essid, sizeof(self->links[i].essid));
    self->links[i].mode = data->links[i].mode;
    self->links[i].encrypt = data->links[i].encrypt;
    self->links[i].freq = data->links[i].freq;
    self->links[i].qual = data->links[i].qual;
    self->links[i].level = data->links[i].level;
    self->links[i].noise = data->links[i].noise;
  }
  }
  return;
}

// Get link state
playerc_wifi_link_t *playerc_wifi_get_link(playerc_wifi_t *self, int link)
{
  //if (link >= self->link_count)
  //  return NULL;

  return self->links + link;
}

