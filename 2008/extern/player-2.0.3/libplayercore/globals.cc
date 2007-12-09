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

#ifndef _GLOBALS_H
#define _GLOBALS_H

#if HAVE_CONFIG_H
  #include <config.h>
#endif

#include <string.h>
#include <stdlib.h>
#include <assert.h>

#include <libplayercore/devicetable.h>
#include <libplayercore/drivertable.h>
#include <libplayercore/playertime.h>
#include <libplayercore/wallclocktime.h>

// this table holds all the currently *instantiated* devices
DeviceTable* deviceTable;

// this table holds all the currently *available* drivers
DriverTable* driverTable;

// the global PlayerTime object has a method 
//   int GetTime(struct timeval*)
// which everyone must use to get the current time
PlayerTime* GlobalTime;

char playerversion[32];

bool player_quit;
bool player_quiet_startup;

void
player_globals_init()
{
  deviceTable = new DeviceTable();
  driverTable = new DriverTable();
  GlobalTime = new WallclockTime();
  strncpy(playerversion, VERSION, sizeof(playerversion));
  player_quit = false;
  player_quiet_startup = false;
}

void
player_globals_fini()
{
  if(deviceTable)
    delete deviceTable;
  if(driverTable)
    delete driverTable;
  if(GlobalTime)
    delete GlobalTime;
}

#endif
