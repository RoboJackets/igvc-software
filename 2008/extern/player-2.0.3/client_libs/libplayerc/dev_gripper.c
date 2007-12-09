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
 * Desc: Gripper device proxy
 * Author: Doug Blank
 * Date: 13 April 2005
 * CVS: $Id: dev_gripper.c,v 1.6 2005/10/16 02:06:47 rtv Exp $
 **************************************************************************/

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <netinet/in.h>

#include "playerc.h"
#include "error.h"

// Local declarations
void playerc_gripper_putmsg(playerc_gripper_t *device, 
			     player_msghdr_t *header,
			     void* generic );

// Create a new gripper proxy
playerc_gripper_t *playerc_gripper_create(playerc_client_t *client, int index)
{
  playerc_gripper_t *device;

  device = malloc(sizeof(playerc_gripper_t));
  memset(device, 0, sizeof(playerc_gripper_t));
  playerc_device_init(&device->info, client, PLAYER_GRIPPER_CODE, index,
                      (playerc_putmsg_fn_t) playerc_gripper_putmsg);
  return device;
}


// Destroy a gripper proxy
void playerc_gripper_destroy(playerc_gripper_t *device)
{
  playerc_device_term(&device->info);
  free(device);
  return;
}


// Subscribe to the gripper device
int playerc_gripper_subscribe(playerc_gripper_t *device, int access)
{
  return playerc_device_subscribe(&device->info, access);
}


// Un-subscribe from the gripper device
int playerc_gripper_unsubscribe(playerc_gripper_t *device)
{
  return playerc_device_unsubscribe(&device->info);
}


// Process incoming data
void playerc_gripper_putmsg(playerc_gripper_t *device, 
			     player_msghdr_t *header,
			     void* generic )
{
  
  if( header->type == PLAYER_MSGTYPE_DATA &&
      header->subtype == PLAYER_GRIPPER_DATA_STATE )
  {
    player_gripper_data_t * data = (player_gripper_data_t * ) generic;

    device->state = data->state;
    device->beams = data->beams;
    device->outer_break_beam = (data->beams & 0x04) ? 1 : 0;
    device->inner_break_beam = (data->beams & 0x08) ? 1 : 0;
    device->paddles_open = (device->state & 0x01) ? 1 : 0;
    device->paddles_closed = (device->state & 0x02) ? 1 : 0;
    device->paddles_moving = (device->state & 0x04) ? 1 : 0;
    device->gripper_error = (device->state & 0x08) ? 1 : 0;
    device->lift_up = (device->state & 0x10) ? 1 : 0;
    device->lift_down = (device->state & 0x20) ? 1 : 0;
    device->lift_moving = (device->state & 0x40) ? 1 : 0;
    device->lift_error = (device->state & 0x80) ? 1 : 0;
  }
}

// Set a cmd
int playerc_gripper_set_cmd(playerc_gripper_t *device, 
			    uint8_t command, 
			    uint8_t arg)
{
  player_gripper_cmd_t cmd;

  memset(&cmd, 0, sizeof(cmd));
  cmd.cmd = command;
  cmd.arg = arg;
  
  return playerc_client_write(device->info.client, &device->info, 
			      PLAYER_GRIPPER_CMD_STATE, 
			      &cmd, NULL );
}


// Get the geometry.  The writes the result into the proxy
// rather than returning it to the caller.
int
playerc_gripper_get_geom(playerc_gripper_t *device)
{
  player_gripper_geom_t config;
  
  if(playerc_client_request(device->info.client, 
                            &device->info,PLAYER_GRIPPER_REQ_GET_GEOM,
                            NULL, &config, sizeof(config)) < 0)
    return -1;

  device->pose[0] = config.pose.px;
  device->pose[1] = config.pose.py;
  device->pose[2] = config.pose.pa;
  device->size[0] = config.size.sl;
  device->size[1] = config.size.sw;
  
  return 0;
}



// print human-readable state 
void playerc_gripper_printout(playerc_gripper_t *device, 
			   const char* prefix )
{
  if( prefix )
    printf( "%s: ", prefix );
  
  printf("[%14.3f]"
	 " pose[%.2f,%.2f,%.2f]"
	 " size[%.2f,%.2f]"
	 " outer_break_beam: %d"
	 " inner_break_beam: %d"
	 " paddles_open: %d"
	 " paddles_closed: %d"
	 " paddles_moving: %d"
	 " gripper_error: %d"
	 " lift_up: %d"
	 " lift_down: %d"
	 " lift_moving: %d"
	 " lift_error: %d\n",	    
	 device->info.datatime, 
	 device->pose[0], device->pose[1], device->pose[2],
	 device->size[0], device->size[1],
	 device->outer_break_beam,
	 device->inner_break_beam,
	 device->paddles_open,
	 device->paddles_closed,
	 device->paddles_moving,
	 device->gripper_error,
	 device->lift_up,
	 device->lift_down,
	 device->lift_moving,
	 device->lift_error );	     
}
