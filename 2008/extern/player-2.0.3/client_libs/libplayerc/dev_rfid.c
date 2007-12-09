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
/***************************************************************************
 * Desc: RFID reader proxy
 * Author: Radu Bogdan Rusu
 * Date: 31 January 2006
 **************************************************************************/

#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <netinet/in.h>

#include "playerc.h"
#include "error.h"

// Process incoming data
void playerc_rfid_putmsg (playerc_rfid_t *device, 
                          player_msghdr_t *header,
			  void *data);

// Create a new rfid proxy
playerc_rfid_t *playerc_rfid_create(playerc_client_t *client, int index)
{
    playerc_rfid_t *device;
    device = malloc(sizeof(playerc_rfid_t));
    memset(device, 0, sizeof(playerc_rfid_t));
    playerc_device_init(&device->info, client, PLAYER_RFID_CODE, index,
       (playerc_putmsg_fn_t) playerc_rfid_putmsg);

    return device;
}


// Destroy a rfid proxy
void playerc_rfid_destroy(playerc_rfid_t *device)
{
    playerc_device_term(&device->info);
    free(device);
}


// Subscribe to the rfid device
int playerc_rfid_subscribe(playerc_rfid_t *device, int access)
{
    return playerc_device_subscribe(&device->info, access);
}


// Un-subscribe from the rfid device
int playerc_rfid_unsubscribe(playerc_rfid_t *device)
{
    return playerc_device_unsubscribe(&device->info);
}

// Process incoming data
void playerc_rfid_putmsg (playerc_rfid_t *device, 
			  player_msghdr_t *header,
			  void *data)
{
    int i, j;
	
    if((header->type == PLAYER_MSGTYPE_DATA) &&
       (header->subtype == PLAYER_RFID_DATA))
    {
	player_rfid_data_t* rfid_data = (player_rfid_data_t*)data;
	device->tags_count = rfid_data->tags_count;
	for (i = 0; i < device->tags_count; i++)
	{
	    if (i >= PLAYERC_RFID_MAX_TAGS)
		break;
	    device->tags[i].type = rfid_data->tags[i].type;
	    for (j = 0; j < PLAYERC_RFID_MAX_GUID; j++)
		device->tags[i].guid[j] = rfid_data->tags[i].guid[j];
	}
    }
    else
	PLAYERC_WARN2("skipping rfid message with unknown type/subtype: %d/%d\n",
	    header->type, header->subtype);
}

