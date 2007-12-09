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
 * $Id: playererror.cc,v 1.5 2005/11/08 17:07:00 bradkratochvil Exp $
 *
 *
 */

#include "playererror.h"
#include "debug.h"

using namespace PlayerCc;

PlayerError::PlayerError(const std::string aFun,
                         const std::string aStr,
                         const int aCode) :
  mStr(aStr),
  mFun(aFun),
  mCode(aCode)
{
  LOG(aFun << " " << aStr << " " << aCode);
}

PlayerError::~PlayerError()
{

}

std::ostream& std::operator << (std::ostream& os, const PlayerError& e)
{
  return os << e.GetErrorFun()
            << "(" << e.GetErrorCode() << ")"
            << " : "
            << e.GetErrorStr();
}
