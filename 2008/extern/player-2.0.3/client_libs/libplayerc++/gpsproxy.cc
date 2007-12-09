/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000-2003
 *     Brian Gerkey, Kasper Stoy, Richard Vaughan, & Andrew Howard
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
 * $Id: gpsproxy.cc,v 1.3 2006/02/24 02:46:14 thjc Exp $
 */

#include "playerc++.h"

using namespace PlayerCc;

GpsProxy::GpsProxy(PlayerClient *aPc, uint aIndex)
  : ClientProxy(aPc, aIndex),
  mDevice(NULL)
{
  Subscribe(aIndex);
  // how can I get this into the clientproxy.cc?
  // right now, we're dependent on knowing its device type
  mInfo = &(mDevice->info);
}

GpsProxy::~GpsProxy()
{
  Unsubscribe();
}

void
GpsProxy::Subscribe(uint aIndex)
{
  scoped_lock_t lock(mPc->mMutex);
  mDevice = playerc_gps_create(mClient, aIndex);
  if (NULL==mDevice)
    throw PlayerError("GpsProxy::GpsProxy()", "could not create");

  if (0 != playerc_gps_subscribe(mDevice, PLAYER_OPEN_MODE))
    throw PlayerError("GpsProxy::GpsProxy()", "could not subscribe");
}

void
GpsProxy::Unsubscribe()
{
  assert(NULL!=mDevice);
  scoped_lock_t lock(mPc->mMutex);
  playerc_gps_unsubscribe(mDevice);
  playerc_gps_destroy(mDevice);
  mDevice = NULL;
}

std::ostream&
std::operator << (std::ostream &os, const PlayerCc::GpsProxy &c)
{
  os << "#GPS (" << c.GetInterface() << ":" << c.GetIndex() << ")" << std::endl;
  os << "#lat|long|alt|utm_e|utm_n|err_horz|err_vert|num_sats" << std::endl;
  os << c.GetLatitude() << " " << c.GetLongitude() << " " << c.GetAltitude() << " " ;
  os << c.GetUtmEasting() << " " << c.GetUtmNorthing() << " " << c.GetErrHorizontal() << " ";
  os << c.GetErrVertical() << " " << c.GetSatellites() << std::endl;
  return os;
}
