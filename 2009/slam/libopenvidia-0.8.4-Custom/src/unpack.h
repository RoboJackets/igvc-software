///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2002, Industrial Light & Magic, a division of Lucas
// Digital Ltd. LLC
//
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
// *       Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
// *       Redistributions in binary form must reproduce the above
// copyright notice, this list of conditions and the following disclaimer
// in the documentation and/or other materials provided with the
// distribution.
// *       Neither the name of Industrial Light & Magic nor the names of
// its contributors may be used to endorse or promote products derived
// from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <iomanip>

using namespace std;

//---------------------------------------------------
// Interpret an unsigned short bit pattern as a half,
// and convert that half to the corresponding float's
// bit pattern.
//---------------------------------------------------

unsigned int
halfToFloat (unsigned short y)
{

    int s = (y >> 15) & 0x00000001;
    int e = (y >> 10) & 0x0000001f;
    int m =  y        & 0x000003ff;

    if (e == 0)
    {
        if (m == 0)
        {
            //
            // Plus or minus zero
            //

            return s << 31;
        }
        else
        {
            //
            // Denormalized number -- renormalize it
            //

            while (!(m & 0x00000400))
            {
                m <<= 1;
                e -=  1;
            }

            e += 1;
            m &= ~0x00000400;
        }
    }
    else if (e == 31)
    {
        if (m == 0)
        {
            //
            // Positive or negative infinity
            //
            return (s << 31) | 0x7f800000;
        }
        else
        {
            //
            // Nan -- preserve sign and significand bits
            //

            return (s << 31) | 0x7f800000 | (m << 13);
        }
    }

    //
    // Normalized number
    //

    e = e + (127 - 15);
    m = m << 13;

    //
    // Assemble s, e and m.
    //

    return (s << 31) | (e << 23) | m;
}


/**
 * This next little bit of this particular file is part of the OpenVIDIA project at http://openvidia.sf.net
 * Copyright (C) 2004, James Fung
 *
 * OpenVIDIA is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * OpenVIDIA is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with OpenVIDIA; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 **/

// unpacks 2 half (which were packed into the float "in") and
// palces the results in out1, out2 as floating point (loss of
// precision tho)
void unpack_2half( float in, float *out1, float *out2)
{
    /*
      unsigned int *x;
      unsigned int y;
      float input = in;

      //x = (unsigned int *) &(featureVectors[pos][0][0]);
      x = (unsigned int *) &input;
      y = *x;
      //y = y & 0x0000FFFF;

      unsigned short *a;
      unsigned short b,c;

      a = (unsigned short *)&y;
      b =( y >> 16  ) & 0x0000FFFF;
      c =( y >> 0   ) & 0x0000FFFF;

      unsigned int f = halfToFloat( b );
      unsigned int g = halfToFloat( c );

      float *newFloat_b;
      float *newFloat_c;

      newFloat_b = (float *)&f;
      newFloat_c = (float *)&g;

      //cerr << "new float = " << *newFloat_b<<" and "<< *newFloat_c<<endl;
      *out2 = *newFloat_b;
      *out1 = *newFloat_c;
    */

    unsigned int *myX = (unsigned int *)&in ;
    unsigned int f = halfToFloat( ((*myX)>>16)&0x0000FFFF );
    unsigned int g = halfToFloat( ((*myX)>>0 )&0x0000FFFF );
//  float *newFloat_b;
//  float *newFloat_c;

//  newFloat_b = (float *)&f;
//  newFloat_c = (float *)&g;

    //float x = *newFloat_b;
    //float y = *newFloat_c;

    float x = *(float *)&f;
    float y = *(float *)&g;

    //cerr << "new float = " << *newFloat_b<<" and "<< *newFloat_c<<endl;
    //*out2 = *newFloat_b;
    //*out1 = *newFloat_c;
    *out2 = x;
    *out1 = y;

}

