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
 * $Id: gripperproxy.cc,v 1.5 2005/10/24 09:02:40 bradkratochvil Exp $
 */

#include "playerc++.h"

using namespace PlayerCc;

GripperProxy::GripperProxy(PlayerClient *aPc, uint aIndex)
  : ClientProxy(aPc, aIndex),
  mDevice(NULL)
{
  Subscribe(aIndex);
  // how can I get this into the clientproxy.cc?
  // right now, we're dependent on knowing its device type
  mInfo = &(mDevice->info);
}

GripperProxy::~GripperProxy()
{
  Unsubscribe();
}

void
GripperProxy::Subscribe(uint aIndex)
{
  scoped_lock_t lock(mPc->mMutex);
  mDevice = playerc_gripper_create(mClient, aIndex);
  if (NULL==mDevice)
    throw PlayerError("GripperProxy::GripperProxy()", "could not create");

  if (0 != playerc_gripper_subscribe(mDevice, PLAYER_OPEN_MODE))
    throw PlayerError("GripperProxy::GripperProxy()", "could not subscribe");
}

void
GripperProxy::Unsubscribe()
{
  assert(NULL!=mDevice);
  scoped_lock_t lock(mPc->mMutex);
  playerc_gripper_unsubscribe(mDevice);
  playerc_gripper_destroy(mDevice);
  mDevice = NULL;
}



std::ostream& std::operator << (std::ostream &os, const PlayerCc::GripperProxy &c)
{
  os << "#Gripper (" << c.GetInterface() << ":" << c.GetIndex() << ")" << std::endl;
  os << (c.GetState() ? "open" : "closed") << " ";
  os << (c.GetInnerBreakBeam() ? "broken" : "clear") << " ";
  os << (c.GetOuterBreakBeam() ? "broken" : "clear") << std::endl;

  return os;
}


// send a gripper command
//
// Returns:
//   0 if everything's ok
//   -1 otherwise (that's bad)
void GripperProxy::SetGrip(uint8_t aCmd, uint8_t aArg)
{
  scoped_lock_t lock(mPc->mMutex);
  if (0 != playerc_gripper_set_cmd(mDevice,aCmd,aArg))
    throw PlayerError("GripperProxy::SetGrip()", "error setting grip");
  return;
}

